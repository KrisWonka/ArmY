#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudToBase(object):
    def __init__(self):
        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.input_topic = rospy.get_param("~input_topic", "/rgbd_cam/depth/points")
        self.output_topic = rospy.get_param("~output_topic", "/rgbd_cam/depth/points_base")
        self.transform_timeout = float(rospy.get_param("~transform_timeout_s", 0.20))
        self.use_latest_tf = bool(rospy.get_param("~use_latest_tf", True))
        self.fallback_to_latest_tf = bool(rospy.get_param("~fallback_to_latest_tf", True))
        self.max_input_age_s = float(rospy.get_param("~max_input_age_s", 0.0))
        self.tf_fail_reinit_threshold = int(rospy.get_param("~tf_fail_reinit_threshold", 20))
        self.downsample_step = int(rospy.get_param("~downsample_step", 4))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self._cb, queue_size=1)
        self._seq_in = 0
        self._seq_pub = 0
        self._seq_drop = 0
        self._tf_fail_streak = 0
        rospy.loginfo(
            "pointcloud_to_base: %s -> %s in frame %s (latest_tf=%s fallback_latest=%s timeout=%.2fs max_age=%.2fs)",
            self.input_topic,
            self.output_topic,
            self.target_frame,
            str(self.use_latest_tf).lower(),
            str(self.fallback_to_latest_tf).lower(),
            self.transform_timeout,
            self.max_input_age_s,
        )
        rospy.loginfo("pointcloud_to_base: downsample_step=%d", self.downsample_step)

    def _downsample_cloud(self, msg):
        step = max(1, int(self.downsample_step))
        if step <= 1:
            return msg
        if msg.point_step <= 0 or not msg.data:
            return msg

        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.is_dense = msg.is_dense

        # Organized cloud case (typical depth camera).
        if msg.height > 1 and msg.width > 1 and msg.row_step >= msg.width * msg.point_step:
            new_width = max(1, msg.width // step)
            new_height = max(1, msg.height // step)
            point_step = msg.point_step
            row_step = msg.row_step
            data = msg.data
            out_data = bytearray(new_width * new_height * point_step)
            idx = 0
            for r in range(0, msg.height, step):
                if r // step >= new_height:
                    break
                row_base = r * row_step
                for c in range(0, msg.width, step):
                    if c // step >= new_width:
                        break
                    p0 = row_base + c * point_step
                    p1 = p0 + point_step
                    out_data[idx:idx + point_step] = data[p0:p1]
                    idx += point_step
            out.width = new_width
            out.height = new_height
            out.row_step = new_width * point_step
            out.data = bytes(out_data[:idx])
            return out

        # Unorganized cloud fallback.
        npts = len(msg.data) // msg.point_step
        keep = max(1, npts // step)
        out_data = bytearray(keep * msg.point_step)
        idx = 0
        for i in range(0, npts, step):
            if idx >= len(out_data):
                break
            p0 = i * msg.point_step
            p1 = p0 + msg.point_step
            out_data[idx:idx + msg.point_step] = msg.data[p0:p1]
            idx += msg.point_step
        out.width = max(1, idx // msg.point_step)
        out.height = 1
        out.row_step = out.width * msg.point_step
        out.data = bytes(out_data[:idx])
        return out

    def _reinit_tf_listener(self):
        # Recreate TF buffer/listener to recover from occasional broken TF streams.
        try:
            self.tf_listener = None
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            rospy.logwarn("pointcloud_to_base: reinitialized tf listener")
        except Exception as exc:
            rospy.logwarn("pointcloud_to_base: tf reinit failed: %s", str(exc))

    def _cb(self, msg):
        self._seq_in += 1
        now = rospy.Time.now()
        if msg.header.stamp and self.max_input_age_s > 0.0:
            age = (now - msg.header.stamp).to_sec()
            if age > self.max_input_age_s:
                self._seq_drop += 1
                rospy.logwarn_throttle(
                    2.0,
                    "pointcloud_to_base drop stale input: age=%.3fs in=%d pub=%d drop=%d",
                    age,
                    self._seq_in,
                    self._seq_pub,
                    self._seq_drop,
                )
                return

        query_time = rospy.Time(0) if self.use_latest_tf else msg.header.stamp
        try:
            in_msg = self._downsample_cloud(msg)
            try:
                tfm = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    in_msg.header.frame_id,
                    query_time,
                    rospy.Duration(self.transform_timeout),
                )
            except Exception as first_exc:
                # If exact-time lookup fails, optionally fallback to latest TF.
                if (not self.use_latest_tf) and self.fallback_to_latest_tf:
                    tfm = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        in_msg.header.frame_id,
                        rospy.Time(0),
                        rospy.Duration(self.transform_timeout),
                    )
                    rospy.logwarn_throttle(
                        1.0,
                        "pointcloud_to_base fallback latest TF: %s",
                        str(first_exc),
                    )
                else:
                    raise
            out = do_transform_cloud(in_msg, tfm)
            out.header.frame_id = self.target_frame
            out.header.stamp = now
            self.pub.publish(out)
            self._seq_pub += 1
            self._tf_fail_streak = 0
        except Exception as exc:
            self._seq_drop += 1
            self._tf_fail_streak += 1
            if self._tf_fail_streak >= self.tf_fail_reinit_threshold:
                self._tf_fail_streak = 0
                self._reinit_tf_listener()
            rospy.logwarn_throttle(
                1.0,
                "pointcloud_to_base tf failed(%d): %s | in=%d pub=%d drop=%d",
                self._tf_fail_streak,
                str(exc),
                self._seq_in,
                self._seq_pub,
                self._seq_drop,
            )


if __name__ == "__main__":
    rospy.init_node("pointcloud_to_base", log_level=rospy.INFO)
    PointCloudToBase()
    rospy.spin()
