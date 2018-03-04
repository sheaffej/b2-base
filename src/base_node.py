#!/usr/bin/env python
from __future__ import print_function
from math import pi, sin, cos
import threading

import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
# from tf.broadcaster import TransformBroadcaster
import tf

from roboclaw_driver.msg import SpeedCommand, Stats

DEFAULT_NODE_NAME = "base_node"
DEFAULT_CMD_TOPIC = "teleop_node/cmd_vel"
DEFAULT_ROBOCLAW_STATS_TOPIC = "roboclaw/stats"
DEFAULT_LOOP_HZ = 10  # hertz
DEFAULT_WHEEL_DIST = 0.220  # meters
DEFAULT_WHEEL_RADIUS = 0.0325  # meters
DEFAULT_TICKS_PER_ROTATION = 48 * 34
DEFAULT_MAX_QPPS = 3700
DEFAULT_MAX_DRIVE_SECS = 1
DEFAULT_ODOM_FRAME_ID = "world"
DEFAULT_BASE_FRAME_ID = "base_link"


class BaseNode:
    def __init__(self, node_name):
        self._node_name = node_name

        self._x_linear_cmd = 0.0
        self._z_angular_cmd = 0.0
        self._cmd_vel_lock = threading.RLock()  # To serialize access to x/z command variables

        self._m1_qpps_actual = 0
        self._m2_qpps_actual = 0
        self._stats_lock = threading.RLock()  # To serialize access to the qpps stats

        self._wheel_dist = rospy.get_param("~wheel_dist", DEFAULT_WHEEL_DIST)
        self._wheel_radius = rospy.get_param("~wheel_radius", DEFAULT_WHEEL_RADIUS)
        self._ticks_per_radian = (
            rospy.get_param("~ticks_per_rotation", DEFAULT_TICKS_PER_ROTATION) / (2 * pi)
        )
        self._max_drive_secs = rospy.get_param("~max_drive_secs", DEFAULT_MAX_DRIVE_SECS)
        self._base_frame_id = rospy.get_param("~base_frame_id", DEFAULT_BASE_FRAME_ID)
        self._world_frame_id = rospy.get_param("~odom_frame_id", DEFAULT_ODOM_FRAME_ID)

        self._world_x = 0.0
        self._world_y = 0.0
        self._world_theta = 0.0
        self._last_odom_time = rospy.get_rostime()

        # Set up the Joy message Subscriber
        rospy.Subscriber(
            rospy.get_param("~cmd_topic", DEFAULT_CMD_TOPIC),
            Twist,
            self._cmd_vel_callback
        )

        # Set up the Roboclaw Stats message Subscriber
        rospy.Subscriber(
            rospy.get_param("~roboclaw_stats_topic", DEFAULT_ROBOCLAW_STATS_TOPIC),
            Stats,
            self._roboclaw_stats_callback
        )

        # Set up the publishers
        self._speed_cmd_pub = rospy.Publisher('~speed_command', SpeedCommand, queue_size=1)
        self._odom_pub = rospy.Publisher('~odom', Odometry, queue_size=1)
        self._tf_broadcaster = tf.broadcaster.TransformBroadcaster()

    def run(self):
        """Runs the main loop of the node.
        Sends motor commands, and publishes odometry.
        """
        rospy.loginfo("Running node")
        looprate = rospy.Rate(rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ))

        try:
            while not rospy.is_shutdown():

                # ---------------------------------
                # Calculate and send motor commands
                # ---------------------------------
                right_angular_v = (
                    (self._x_linear_cmd + self._z_angular_cmd * (self._wheel_dist / 2.0))
                    / self._wheel_radius
                )
                left_angular_v = (
                    (self._x_linear_cmd - self._z_angular_cmd * (self._wheel_dist / 2.0))
                    / self._wheel_radius
                )

                right_qpps_target = right_angular_v * self._ticks_per_radian
                left_qpps_target = left_angular_v * self._ticks_per_radian

                cmd = SpeedCommand()
                cmd.m1_qpps = right_qpps_target
                cmd.m2_qpps = left_qpps_target
                cmd.max_secs = self._max_drive_secs
                self._speed_cmd_pub.publish(cmd)

                # ----------------
                # Publish Odometry
                # ----------------
                right_angular_v = self._m1_qpps_actual / self._ticks_per_radian
                left_angular_v = self._m2_qpps_actual / self._ticks_per_radian

                right_linear_v = right_angular_v * self._wheel_radius
                left_linear_v = left_angular_v * self._wheel_radius

                x_linear_v = (right_linear_v + left_linear_v) / 2
                # y_linear_v = 0  # Because the robot is nonholonomic
                z_angular_v = (right_linear_v - left_linear_v) / self._wheel_dist

                # 2D rotation matrix math https://en.wikipedia.org/wiki/Rotation_matrix
                # But since y_linear_v = 0, we don't need the second part of each equation
                world_x_velocity = x_linear_v * cos(self._world_theta)  # - y_linear_v * sin(z_angular_v)
                world_y_velocity = x_linear_v * sin(self._world_theta)  # + y_linear_v * cos(z_angular_v)
                world_angular_velocity = z_angular_v

                now = rospy.get_rostime()
                time_delta_secs = (now - self._last_odom_time).to_sec()
                self._last_odom_time = now

                self._world_x = self._world_x + (world_x_velocity * time_delta_secs)
                self._world_y = self._world_y + (world_y_velocity * time_delta_secs)
                self._world_theta = self._world_theta + (world_angular_velocity * time_delta_secs)

                # Convert world orientation (theta) to a Quaternion for use with tf and Odometry
                quat_vals = tf.transformations.quaternion_from_euler(0, 0, self._world_theta)
                quat = Quaternion()
                quat.x = quat_vals[0]
                quat.y = quat_vals[1]
                quat.z = quat_vals[2]
                quat.w = quat_vals[3]

                odom = Odometry()
                odom.header.stamp = now
                odom.header.frame_id = self._world_frame_id
                odom.pose.pose.position.x = self._world_x
                odom.pose.pose.position.y = self._world_y
                odom.pose.pose.position.z = 0
                odom.pose.pose.orientation = quat
                odom.child_frame_id = self._base_frame_id
                odom.twist.twist.linear.x = x_linear_v
                odom.twist.twist.linear.y = 0
                odom.twist.twist.angular.z = z_angular_v

                self._odom_pub.publish(odom)

                # Broadcast tf transform for other nodes to use
                self._tf_broadcaster.sendTransform(
                    (self._world_x, self._world_y, 0),
                    (quat.x, quat.y, quat.z, quat.w),
                    rospy.Time.now(),
                    self._base_frame_id,
                    self._world_frame_id
                )

                looprate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ROSInterruptException received in main loop")

    def _cmd_vel_callback(self, msg):
        """Called by the Twist cmd_vel message subscriber.

        Parameters:
            :param Twist msg: Twist command velocity message
        """
        with self._cmd_vel_lock:
            self._x_linear_cmd = msg.linear.x
            self._z_angular_cmd = msg.angular.z

    def _roboclaw_stats_callback(self, stats):
        """Called by the Roboclaw Stats message subscriber

        Parameters:
            :param Stats stats: Roboclaw Stas message
        """
        with self._stats_lock:
            self._m1_qpps_actual = stats.m1_enc_qpps
            self._m2_qpps_actual = stats.m2_enc_qpps


if __name__ == "__main__":
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    node = BaseNode(node_name)
    node.run()
