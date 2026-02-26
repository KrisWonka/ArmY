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

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self._cb, queue_size=1)
        rospy.loginfo(
            "pointcloud_to_base: %s -> %s in frame %s",
            self.input_topic,
            self.output_topic,
            self.target_frame,
        )

    def _cb(self, msg):
        try:
            tfm = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration(0.08),
            )
            out = do_transform_cloud(msg, tfm)
            out.header.frame_id = self.target_frame
            self.pub.publish(out)
        except Exception as exc:
            # Fallback for timestamp skew: use latest available transform.
            try:
                tfm = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.05),
                )
                out = do_transform_cloud(msg, tfm)
                out.header.frame_id = self.target_frame
                self.pub.publish(out)
            except Exception:
                rospy.logwarn_throttle(2.0, "pointcloud_to_base tf failed: %s", str(exc))


if __name__ == "__main__":
    rospy.init_node("pointcloud_to_base", log_level=rospy.INFO)
    PointCloudToBase()
    rospy.spin()
