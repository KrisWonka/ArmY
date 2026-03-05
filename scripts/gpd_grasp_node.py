#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import os
import yaml
import math
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from hiwonder_interfaces.msg import (
    Grasp,
    MoveAction,
    MoveGoal,
    MultiRawIdPosDur,
    RawIdPosDur,
)

try:
    from gpd_ros.msg import GraspConfigList
    HAS_GPD = True
except Exception:
    HAS_GPD = False


class GpdGraspNode:
    def __init__(self):
        rospy.init_node("gpd_grasp")
        self.grasp_topic = rospy.get_param("~grasp_topic", "/detect_grasps/clustered_grasps")
        self.pitch = rospy.get_param("~pitch", 80.0)
        self.align_angle = rospy.get_param("~align_angle", 0.0)
        self.pre_grasp_posture = rospy.get_param("~pre_grasp_posture", 600)
        self.grasp_posture = rospy.get_param("~grasp_posture", 400)
        self.approach = rospy.get_param("~approach", {"x": 0.0, "y": 0.0, "z": 0.02})
        self.retreat = rospy.get_param("~retreat", {"x": 0.0, "y": 0.0, "z": 0.03})
        # Slanted push-pick:
        # angle is measured from horizontal plane, positive means downward.
        self.use_slanted_push_pick = bool(rospy.get_param("~use_slanted_push_pick", True))
        self.attack_angle_deg = float(rospy.get_param("~attack_angle_deg", 20.0))
        self.base_pick_pitch_deg = float(rospy.get_param("~base_pick_pitch_deg", 80.0))
        self.approach_distance_m = float(rospy.get_param("~approach_distance_m", 0.05))
        self.retreat_back_m = float(rospy.get_param("~retreat_back_m", 0.03))
        self.retreat_up_m = float(rospy.get_param("~retreat_up_m", 0.04))
        self.x_offset = rospy.get_param("~x_offset", 0.0)
        self.y_offset = rospy.get_param("~y_offset", 0.0)
        self.z_offset = rospy.get_param("~z_offset", 0.0)

        # Keep only candidates in a conservative reachable box.
        self.pick_bounds = rospy.get_param(
            "~pick_bounds",
            {
                "x_min": -0.30,
                "x_max": 0.30,
                "y_min": -0.25,
                "y_max": 0.25,
                "z_min": 0.00,
                "z_max": 0.25,
            },
        )
        # Prefer grasps near the workspace center to avoid edge/noisy picks.
        self.preferred_point = rospy.get_param(
            "~preferred_point",
            {"x": 0.18, "y": 0.00, "z": 0.05},
        )
        self.return_to_init = rospy.get_param("~return_to_init", True)
        self.return_duration_ms = int(rospy.get_param("~return_duration_ms", 900))
        self.init_joints = rospy.get_param(
            "~init_joints",
            [[1, 500], [2, 560], [3, 130], [4, 115], [5, 500], [10, 200]],
        )
        self.ui_config_path = rospy.get_param(
            "~ui_config_path",
            "/home/hiwonder/jetarm/src/jetarm_ui/config/ui_config.yaml",
        )
        self.use_ui_init_pose = rospy.get_param("~use_ui_init_pose", True)
        self.use_saved_grasp_offset = rospy.get_param("~use_saved_grasp_offset", True)
        self.grasp_offset_config_path = rospy.get_param(
            "~grasp_offset_config_path",
            "/home/hiwonder/jetarm/src/jetarm_ui/config/tf_calibration.yaml",
        )
        # Prevent reusing stale grasps from previous round.
        self.grasp_max_age_s = float(rospy.get_param("~grasp_max_age_s", 2.0))
        self.debug_marker_frame = rospy.get_param("~debug_marker_frame", "base_link")
        self.debug_marker_topic = rospy.get_param("~debug_marker_topic", "~debug_markers")
        self.debug_marker_rate_hz = float(rospy.get_param("~debug_marker_rate_hz", 2.0))
        self._sync_init_pose_from_ui_config()
        self._sync_grasp_offset_from_config()

        self.last_grasps = None
        self.last_grasps_recv_time = None
        self.action_client = actionlib.SimpleActionClient("/grasp", MoveAction)
        self.action_client.wait_for_server(rospy.Duration(5.0))
        self.joints_pub = rospy.Publisher(
            "/controllers/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1
        )
        self.debug_markers_pub = rospy.Publisher(
            self.debug_marker_topic, MarkerArray, queue_size=1, latch=True
        )

        if HAS_GPD:
            rospy.Subscriber(self.grasp_topic, GraspConfigList, self._on_grasps, queue_size=1)
            rospy.loginfo("gpd_grasp: subscribed to %s", self.grasp_topic)
        else:
            rospy.logwarn("gpd_grasp: gpd_ros not available, grasp topic disabled")

        self.trigger_srv = rospy.Service("~trigger", Trigger, self._on_trigger)
        self.clear_cache_srv = rospy.Service("~clear_cache", Trigger, self._on_clear_cache)
        self.debug_timer = rospy.Timer(
            rospy.Duration(max(0.1, 1.0 / max(0.1, self.debug_marker_rate_hz))),
            self._publish_debug_markers,
        )
        rospy.loginfo("gpd_grasp: ready")

    def _sync_init_pose_from_ui_config(self):
        if not self.use_ui_init_pose:
            return
        if not os.path.exists(self.ui_config_path):
            rospy.logwarn("gpd_grasp: ui config not found: %s", self.ui_config_path)
            return
        try:
            with open(self.ui_config_path, "r") as f:
                cfg = yaml.safe_load(f) or {}
            init_pose = (cfg.get("poses", {}) or {}).get("init", {}) or {}
            joints = init_pose.get("joints", None)
            duration_ms = init_pose.get("duration_ms", None)
            if isinstance(joints, list) and len(joints) > 0:
                self.init_joints = joints
            if duration_ms is not None:
                self.return_duration_ms = int(duration_ms)
            rospy.loginfo(
                "gpd_grasp: loaded init pose from ui config (%s joints, %d ms)",
                len(self.init_joints),
                self.return_duration_ms,
            )
        except Exception as exc:
            rospy.logwarn("gpd_grasp: load ui init pose failed: %s", str(exc))

    def _on_grasps(self, msg):
        self.last_grasps = msg
        self.last_grasps_recv_time = rospy.Time.now()

    def _on_clear_cache(self, _req):
        self.last_grasps = None
        self.last_grasps_recv_time = None
        return TriggerResponse(success=True, message="grasp cache cleared")

    def _sync_grasp_offset_from_config(self):
        if not self.use_saved_grasp_offset:
            return
        if not os.path.exists(self.grasp_offset_config_path):
            return
        try:
            with open(self.grasp_offset_config_path, "r") as f:
                cfg = yaml.safe_load(f) or {}
            go = cfg.get("grasp_offset", {}) or {}
            if "x" in go:
                self.x_offset = float(go["x"])
            if "y" in go:
                self.y_offset = float(go["y"])
            if "z" in go:
                self.z_offset = float(go["z"])
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "gpd_grasp: load grasp_offset failed: %s", str(exc))

    def _build_slanted_vectors(self):
        # Clamp to requested range: horizontal (0 deg) to 30 deg downward.
        angle_deg = max(0.0, min(30.0, float(self.attack_angle_deg)))
        d = max(0.01, float(self.approach_distance_m))
        ang = math.radians(angle_deg)

        # target1 = target + approach, then move_target goes to target.
        # To "push forward and downward", approach should start from "back and up".
        # In JetArm base frame we treat +x as forward.
        app_x = -d * math.cos(ang)
        app_z = d * math.sin(ang)

        ret_back = max(0.0, float(self.retreat_back_m))
        ret_up = max(0.0, float(self.retreat_up_m))
        ret_x = -ret_back
        ret_z = ret_up

        return angle_deg, {"x": app_x, "y": 0.0, "z": app_z}, {"x": ret_x, "y": 0.0, "z": ret_z}

    def _publish_debug_markers(self, _evt):
        b = self.pick_bounds
        x_min = float(b.get("x_min", -0.30))
        x_max = float(b.get("x_max", 0.30))
        y_min = float(b.get("y_min", -0.25))
        y_max = float(b.get("y_max", 0.25))
        z_min = float(b.get("z_min", 0.00))
        z_max = float(b.get("z_max", 0.25))

        cx = 0.5 * (x_min + x_max)
        cy = 0.5 * (y_min + y_max)
        cz = 0.5 * (z_min + z_max)
        sx = max(0.001, x_max - x_min)
        sy = max(0.001, y_max - y_min)
        sz = max(0.001, z_max - z_min)

        arr = MarkerArray()

        m_box = Marker()
        m_box.header.frame_id = self.debug_marker_frame
        m_box.header.stamp = rospy.Time.now()
        m_box.ns = "gpd_grasp"
        m_box.id = 1
        m_box.type = Marker.CUBE
        m_box.action = Marker.ADD
        m_box.pose.position.x = cx
        m_box.pose.position.y = cy
        m_box.pose.position.z = cz
        m_box.pose.orientation.w = 1.0
        m_box.scale.x = sx
        m_box.scale.y = sy
        m_box.scale.z = sz
        m_box.color.r = 0.1
        m_box.color.g = 0.9
        m_box.color.b = 0.1
        m_box.color.a = 0.18
        arr.markers.append(m_box)

        m_pref = Marker()
        m_pref.header.frame_id = self.debug_marker_frame
        m_pref.header.stamp = rospy.Time.now()
        m_pref.ns = "gpd_grasp"
        m_pref.id = 2
        m_pref.type = Marker.SPHERE
        m_pref.action = Marker.ADD
        m_pref.pose.position.x = float(self.preferred_point.get("x", 0.18))
        m_pref.pose.position.y = float(self.preferred_point.get("y", 0.0))
        m_pref.pose.position.z = float(self.preferred_point.get("z", 0.05))
        m_pref.pose.orientation.w = 1.0
        m_pref.scale.x = 0.025
        m_pref.scale.y = 0.025
        m_pref.scale.z = 0.025
        m_pref.color.r = 0.95
        m_pref.color.g = 0.25
        m_pref.color.b = 0.25
        m_pref.color.a = 0.95
        arr.markers.append(m_pref)

        if self.use_slanted_push_pick:
            angle_deg, app, _ret = self._build_slanted_vectors()
        else:
            angle_deg = 0.0
            app = self.approach

        start = Point()
        start.x = float(self.preferred_point.get("x", 0.18))
        start.y = float(self.preferred_point.get("y", 0.0))
        start.z = float(self.preferred_point.get("z", 0.05))
        end = Point()
        end.x = start.x - float(app.get("x", 0.0))
        end.y = start.y - float(app.get("y", 0.0))
        end.z = start.z - float(app.get("z", 0.0))

        m_arrow = Marker()
        m_arrow.header.frame_id = self.debug_marker_frame
        m_arrow.header.stamp = rospy.Time.now()
        m_arrow.ns = "gpd_grasp"
        m_arrow.id = 3
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        m_arrow.points = [start, end]
        m_arrow.scale.x = 0.006
        m_arrow.scale.y = 0.012
        m_arrow.scale.z = 0.015
        m_arrow.color.r = 0.2
        m_arrow.color.g = 0.5
        m_arrow.color.b = 1.0
        m_arrow.color.a = 0.95
        arr.markers.append(m_arrow)

        m_text = Marker()
        m_text.header.frame_id = self.debug_marker_frame
        m_text.header.stamp = rospy.Time.now()
        m_text.ns = "gpd_grasp"
        m_text.id = 4
        m_text.type = Marker.TEXT_VIEW_FACING
        m_text.action = Marker.ADD
        m_text.pose.position.x = cx
        m_text.pose.position.y = cy
        m_text.pose.position.z = z_max + 0.04
        m_text.pose.orientation.w = 1.0
        m_text.scale.z = 0.02
        m_text.color.r = 1.0
        m_text.color.g = 1.0
        m_text.color.b = 1.0
        m_text.color.a = 0.95
        m_text.text = "pick_bounds | attack={:.1f}deg".format(angle_deg)
        arr.markers.append(m_text)

        self.debug_markers_pub.publish(arr)

    def _in_pick_bounds(self, p):
        b = self.pick_bounds
        return (
            b.get("x_min", -0.30) <= p.x <= b.get("x_max", 0.30)
            and b.get("y_min", -0.25) <= p.y <= b.get("y_max", 0.25)
            and b.get("z_min", 0.00) <= p.z <= b.get("z_max", 0.25)
        )

    def _score(self, p):
        c = self.preferred_point
        dx = p.x - c.get("x", 0.18)
        dy = p.y - c.get("y", 0.0)
        dz = p.z - c.get("z", 0.05)
        return dx * dx + dy * dy + dz * dz

    def _select_grasp(self):
        candidates = []
        for g in self.last_grasps.grasps:
            if not hasattr(g, "position"):
                continue
            p = g.position
            if self._in_pick_bounds(p):
                raw_score = 0.0
                if hasattr(g, "score") and hasattr(g.score, "data"):
                    raw_score = float(g.score.data)
                # Primary key: classifier score (higher is better).
                # Secondary key: distance to preferred point (smaller is better).
                candidates.append((raw_score, -self._score(p), g))
        if not candidates:
            return None
        candidates.sort(key=lambda x: (x[0], x[1]), reverse=True)
        return candidates[0][2]

    def _move_to_init_pose(self):
        self._sync_init_pose_from_ui_config()
        if not self.return_to_init:
            return
        if not self.init_joints:
            return
        msg = MultiRawIdPosDur()
        msg.id_pos_dur_list = []
        for pair in self.init_joints:
            if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                continue
            sid = int(pair[0])
            pos = int(pair[1])
            pos = max(0, min(1000, pos))
            item = RawIdPosDur()
            item.id = sid
            item.position = pos
            item.duration = self.return_duration_ms
            msg.id_pos_dur_list.append(item)
        if msg.id_pos_dur_list:
            self.joints_pub.publish(msg)

    def _on_trigger(self, _req):
        self._sync_init_pose_from_ui_config()
        self._sync_grasp_offset_from_config()
        if not HAS_GPD:
            return TriggerResponse(success=False, message="gpd_ros not available")
        if self.last_grasps is None or len(self.last_grasps.grasps) == 0:
            return TriggerResponse(success=False, message="no grasps received")
        if self.last_grasps_recv_time is None:
            return TriggerResponse(success=False, message="no grasp timestamp")
        age = (rospy.Time.now() - self.last_grasps_recv_time).to_sec()
        if age > self.grasp_max_age_s:
            return TriggerResponse(
                success=False,
                message="stale grasps: age {:.2f}s > {:.2f}s".format(age, self.grasp_max_age_s),
            )

        grasp_cfg = self._select_grasp()
        if grasp_cfg is None:
            return TriggerResponse(success=False, message="no grasps in pick_bounds")
        pos = grasp_cfg.position if hasattr(grasp_cfg, "position") else Point()
        gscore = 0.0
        if hasattr(grasp_cfg, "score") and hasattr(grasp_cfg.score, "data"):
            gscore = float(grasp_cfg.score.data)

        goal = MoveGoal()
        goal.grasp.mode = "pick"
        goal.grasp.position.x = pos.x + float(self.x_offset)
        goal.grasp.position.y = pos.y + float(self.y_offset)
        goal.grasp.position.z = pos.z + float(self.z_offset)
        if self.use_slanted_push_pick:
            angle_deg, app, ret = self._build_slanted_vectors()
            # Keep IK-friendly pitch near legacy pick pitch, then apply a bounded
            # downward tilt offset from that baseline.
            pitch_cmd = float(self.base_pick_pitch_deg) - float(angle_deg)
            pitch_cmd = max(45.0, min(85.0, pitch_cmd))
            goal.grasp.pitch = pitch_cmd
            goal.grasp.grasp_approach.x = float(app.get("x", 0.0))
            goal.grasp.grasp_approach.y = float(app.get("y", 0.0))
            goal.grasp.grasp_approach.z = float(app.get("z", 0.0))
            goal.grasp.grasp_retreat.x = float(ret.get("x", 0.0))
            goal.grasp.grasp_retreat.y = float(ret.get("y", 0.0))
            goal.grasp.grasp_retreat.z = float(ret.get("z", 0.0))
        else:
            goal.grasp.pitch = float(self.pitch)
            goal.grasp.grasp_approach.x = float(self.approach.get("x", 0.0))
            goal.grasp.grasp_approach.y = float(self.approach.get("y", 0.0))
            goal.grasp.grasp_approach.z = float(self.approach.get("z", 0.02))
            goal.grasp.grasp_retreat.x = float(self.retreat.get("x", 0.0))
            goal.grasp.grasp_retreat.y = float(self.retreat.get("y", 0.0))
            goal.grasp.grasp_retreat.z = float(self.retreat.get("z", 0.03))
        goal.grasp.align_angle = float(self.align_angle)
        goal.grasp.pre_grasp_posture = int(self.pre_grasp_posture)
        goal.grasp.grasp_posture = int(self.grasp_posture)
        rospy.loginfo(
            "gpd_grasp: pick raw(%.4f, %.4f, %.4f) score=%.3f -> cmd(%.4f, %.4f, %.4f), pitch=%.1f, app(%.3f,%.3f,%.3f), pre=%d grasp=%d",
            pos.x,
            pos.y,
            pos.z,
            gscore,
            goal.grasp.position.x,
            goal.grasp.position.y,
            goal.grasp.position.z,
            goal.grasp.pitch,
            goal.grasp.grasp_approach.x,
            goal.grasp.grasp_approach.y,
            goal.grasp.grasp_approach.z,
            goal.grasp.pre_grasp_posture,
            goal.grasp.grasp_posture,
        )

        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        # Consume this batch once to avoid accidentally reusing it.
        self.last_grasps = None
        self.last_grasps_recv_time = None
        self._move_to_init_pose()
        return TriggerResponse(success=True, message="grasp sent, return init issued")


if __name__ == "__main__":
    GpdGraspNode()
    rospy.spin()
