#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import os
import yaml
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Point

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
        self._sync_init_pose_from_ui_config()
        self._sync_grasp_offset_from_config()

        self.last_grasps = None
        self.action_client = actionlib.SimpleActionClient("/grasp", MoveAction)
        self.action_client.wait_for_server(rospy.Duration(5.0))
        self.joints_pub = rospy.Publisher(
            "/controllers/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1
        )

        if HAS_GPD:
            rospy.Subscriber(self.grasp_topic, GraspConfigList, self._on_grasps, queue_size=1)
            rospy.loginfo("gpd_grasp: subscribed to %s", self.grasp_topic)
        else:
            rospy.logwarn("gpd_grasp: gpd_ros not available, grasp topic disabled")

        self.trigger_srv = rospy.Service("~trigger", Trigger, self._on_trigger)
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
                candidates.append((self._score(p), g))
        if not candidates:
            return None
        candidates.sort(key=lambda x: x[0])
        return candidates[0][1]

    def _move_to_init_pose(self):
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
        self._sync_grasp_offset_from_config()
        if not HAS_GPD:
            return TriggerResponse(success=False, message="gpd_ros not available")
        if self.last_grasps is None or len(self.last_grasps.grasps) == 0:
            return TriggerResponse(success=False, message="no grasps received")

        grasp_cfg = self._select_grasp()
        if grasp_cfg is None:
            return TriggerResponse(success=False, message="no grasps in pick_bounds")
        pos = grasp_cfg.position if hasattr(grasp_cfg, "position") else Point()

        goal = MoveGoal()
        goal.grasp.mode = "pick"
        goal.grasp.position.x = pos.x + float(self.x_offset)
        goal.grasp.position.y = pos.y + float(self.y_offset)
        goal.grasp.position.z = pos.z + float(self.z_offset)
        goal.grasp.pitch = float(self.pitch)
        goal.grasp.align_angle = float(self.align_angle)
        goal.grasp.grasp_approach.x = float(self.approach.get("x", 0.0))
        goal.grasp.grasp_approach.y = float(self.approach.get("y", 0.0))
        goal.grasp.grasp_approach.z = float(self.approach.get("z", 0.02))
        goal.grasp.grasp_retreat.x = float(self.retreat.get("x", 0.0))
        goal.grasp.grasp_retreat.y = float(self.retreat.get("y", 0.0))
        goal.grasp.grasp_retreat.z = float(self.retreat.get("z", 0.03))
        goal.grasp.pre_grasp_posture = int(self.pre_grasp_posture)
        goal.grasp.grasp_posture = int(self.grasp_posture)
        rospy.loginfo(
            "gpd_grasp: pick raw(%.4f, %.4f, %.4f) -> cmd(%.4f, %.4f, %.4f), pre=%d grasp=%d",
            pos.x,
            pos.y,
            pos.z,
            goal.grasp.position.x,
            goal.grasp.position.y,
            goal.grasp.position.z,
            goal.grasp.pre_grasp_posture,
            goal.grasp.grasp_posture,
        )

        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        self._move_to_init_pose()
        return TriggerResponse(success=True, message="grasp sent, return init issued")


if __name__ == "__main__":
    GpdGraspNode()
    rospy.spin()
