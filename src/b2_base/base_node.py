#!/usr/bin/env python
from __future__ import print_function
from math import pi, sin, cos
import threading
import math

import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import tf

from roboclaw_driver.msg import SpeedCommand, Stats

DEFAULT_NODE_NAME = "base_node"
DEFAULT_CMD_TOPIC = "base_node/cmd_vel"
DEFAULT_ROBOCLAW_STATS_TOPIC = "roboclaw/stats"
DEFAULT_SPEED_CMD_TOPIC = "roboclaw/speed_command"
DEFAULT_ODOM_TOPIC = "~odom"
DEFAULT_LOOP_HZ = 10  # hertz
DEFAULT_WHEEL_DIST = 0.220  # meters
DEFAULT_WHEEL_RADIUS = 0.0325  # meters
DEFAULT_TICKS_PER_ROTATION = 48 * 34
DEFAULT_MAX_QPPS = 3700
DEFAULT_MAX_DRIVE_SECS = 1
DEFAULT_ODOM_FRAME_ID = "world"
DEFAULT_BASE_FRAME_ID = "base_link"
DEFAULT_DEADMAN_SECS = 1


class BaseNode:
    def __init__(self):

        self._wheel_dist = rospy.get_param("~wheel_dist", DEFAULT_WHEEL_DIST)
        self._wheel_radius = rospy.get_param("~wheel_radius", DEFAULT_WHEEL_RADIUS)
        self._ticks_per_rotation = rospy.get_param(
            "~ticks_per_rotation", DEFAULT_TICKS_PER_ROTATION)
        self._max_drive_secs = rospy.get_param("~max_drive_secs", DEFAULT_MAX_DRIVE_SECS)
        self._max_qpps = rospy.get_param("~max_qpps", DEFAULT_MAX_QPPS)
        self._base_frame_id = rospy.get_param("~base_frame_id", DEFAULT_BASE_FRAME_ID)
        self._world_frame_id = rospy.get_param("~odom_frame_id", DEFAULT_ODOM_FRAME_ID)
        self._deadman_secs = rospy.get_param("~deadman_secs", DEFAULT_DEADMAN_SECS)

        # Init Twist command state
        self._x_linear_cmd = 0.0
        self._z_angular_cmd = 0.0
        self._last_cmd_vel_time = rospy.get_rostime()

        # Init Odometry state
        self._world_x = 0.0
        self._world_y = 0.0
        self._world_theta = 0.0
        # self._last_odom_time = rospy.get_rostime()
        self._roboclaw_stats = Stats()

        self._stats_lock = threading.RLock()  # To serialize access to the qpps stats
        self._cmd_vel_lock = threading.RLock()  # To serialize access to x/z command variables

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
        self._speed_cmd_pub = rospy.Publisher(
            rospy.get_param('~speed_cmd_topic', DEFAULT_SPEED_CMD_TOPIC),
            SpeedCommand,
            queue_size=1
        )
        self._odom_pub = rospy.Publisher(
            rospy.get_param('~odom_topic', DEFAULT_ODOM_TOPIC),
            Odometry,
            queue_size=1
        )
        self._tf_broadcaster = tf.broadcaster.TransformBroadcaster()

    def run(self):
        """Runs the main loop of the node.
        Sends motor commands, and publishes odometry.
        """
        rospy.loginfo("Running node")
        looprate = rospy.Rate(rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ))

        # Set initial states
        m1_enc_prev = self._roboclaw_stats.m1_enc_val
        m2_enc_prev = self._roboclaw_stats.m2_enc_val
        last_odom_time = rospy.get_rostime()

        try:
            while not rospy.is_shutdown():

                # If the last command was over 1 sec ago, stop the base
                if (rospy.get_rostime() - self._last_cmd_vel_time).to_sec() > self._deadman_secs:
                    self._x_linear_cmd = 0.0
                    self._z_angular_cmd = 0.0

                # ---------------------------------
                # Calculate and send motor commands
                # ---------------------------------
                with self._cmd_vel_lock:
                    x_linear_cmd = self._x_linear_cmd
                    z_angular_cmd = self._z_angular_cmd
                cmd = calc_create_speed_cmd(
                    x_linear_cmd, z_angular_cmd,
                    self._wheel_dist, self._wheel_radius,
                    self._ticks_per_rotation, self._max_drive_secs, self._max_qpps
                )
                self._speed_cmd_pub.publish(cmd)

                # ----------------
                # Publish Odometry
                # ----------------
                with self._stats_lock:
                    # Calculate change in encoder readings
                    m1_enc_diff = self._roboclaw_stats.m1_enc_val - m1_enc_prev
                    m2_enc_diff = self._roboclaw_stats.m2_enc_val - m2_enc_prev

                    m1_enc_prev = self._roboclaw_stats.m1_enc_val
                    m2_enc_prev = self._roboclaw_stats.m2_enc_val

                    nowtime = self._roboclaw_stats.header.stamp

                odom = calc_create_odometry(
                    m1_enc_diff, m2_enc_diff, self._ticks_per_rotation,
                    self._wheel_dist, self._wheel_radius,
                    self._world_x, self._world_y, self._world_theta,
                    last_odom_time,
                    self._base_frame_id, self._world_frame_id,
                    nowtime
                )
                self._world_x = odom.pose.pose.position.x
                self._world_y = odom.pose.pose.position.y
                self._world_theta = yaw_from_odom_message(odom)
                self._odom_pub.publish(odom)

                # Broadcast tf transform for other nodes to use
                quat = odom.pose.pose.orientation
                self._tf_broadcaster.sendTransform(
                    (self._world_x, self._world_y, 0),
                    (quat.x, quat.y, quat.z, quat.w),
                    nowtime,
                    self._base_frame_id,
                    self._world_frame_id
                )

                last_odom_time = nowtime
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
            self._last_cmd_vel_time = rospy.get_rostime()

    def _roboclaw_stats_callback(self, stats):
        """Called by the Roboclaw Stats message subscriber

        Parameters:
            :param Stats stats: Roboclaw Stats message
        """
        with self._stats_lock:
            self._roboclaw_stats = stats


def calc_create_speed_cmd(x_linear_cmd, z_angular_cmd, wheel_dist,
                          wheel_radius, ticks_per_rotation, max_drive_secs, max_qpps):
    """Calculate and send motor commands

    Parameters:
        :param double x_linear_cmd: Twist message's linear.x value
        :param double z_angular_cmd: Twist message's angular.z value
        :param double wheel_dist: Distance between wheels (m)
        :param double wheel_radius: Wheel radius (m)
        :param double ticks_per_radian: Number of encoder ticks per radian of wheel rotation
        :param double max_drive_secs: Maximum seconds drive should run before stopping

    Returns: The SpeedCommand message
        :rtype: roboclaw.msg.SpeedCommand
    """
    right_angular_v = (
        (x_linear_cmd + z_angular_cmd * (wheel_dist / 2.0)) / wheel_radius
    )
    left_angular_v = (
        (x_linear_cmd - z_angular_cmd * (wheel_dist / 2.0)) / wheel_radius
    )
    # print("left_angular_v: {}".format(left_angular_v))

    ticks_per_radian = ticks_per_rotation / (math.pi * 2)
    # print("ticks_per_radian: {}".format(ticks_per_radian))

    right_qpps_target = right_angular_v * ticks_per_radian
    left_qpps_target = left_angular_v * ticks_per_radian
    # print("left_qpps_target: {}".format(left_qpps_target))

    # Clamp the target QPPS within the max_qpps
    right_qpps_target = max(-max_qpps, min(right_qpps_target, max_qpps))
    left_qpps_target = max(-max_qpps, min(left_qpps_target, max_qpps))
    # print("right_qpps_target (after clamp): {}".format(right_qpps_target))
    # print("left_qpps_target (after clamp): {}".format(left_qpps_target))

    cmd = SpeedCommand()
    cmd.m1_qpps = right_qpps_target
    cmd.m2_qpps = left_qpps_target
    cmd.max_secs = max_drive_secs
    return cmd


def calc_create_odometry(m1_enc_diff, m2_enc_diff, ticks_per_rotation,
                         wheel_dist, wheel_radius, world_x, world_y, world_theta,
                         last_odom_time, base_frame_id, world_frame_id, now_odom_time):
    '''Calculate and publish Odometry

    Parameters:
        :param int m1_enc_diff: The motor 1 encoder change reported from the Roboclaw
        :param int m2_enc_diff: The motor 2 encoder change reported from the Roboclaw
        :param float ticks_per_rotation: Number of encoder ticks per radion of wheel rotation
        :param float wheel_dist: Distance between wheels (m)
        :param float wheel_radius: Wheel radius (m)
        :param float world_x: Previous world x coordinate
        :param float world_y: Previous world y cooridate
        :param float world_theta: Previous angular orientation in the world frame (radians)
        :param datetime.Time last_odom_time: Last time Odometry was calculated
        :param str base_frame_id: Frame ID of the base
        :param str world_frame_id: Frame ID of the world

    Returns: Calculated Odometry and new world coordinates
        as a tuple (Odometry, world_x, world_y, world_theta)
        :rtype: (Odemetry, float, float, float)
    '''
    ticks_per_radian = ticks_per_rotation / (2 * pi)
    # print("ticks_per_radian: {}".format(ticks_per_radian))

    time_delta_secs = (now_odom_time - last_odom_time).to_sec()
    last_odom_time = now_odom_time
    # print("time_delta_secs: {}".format(time_delta_secs))

    if time_delta_secs > 0:
        m1_qpps_actual = m1_enc_diff / float(time_delta_secs)
        m2_qpps_actual = m2_enc_diff / float(time_delta_secs)
    else:
        m1_qpps_actual = m2_qpps_actual = 0

    # print("m1_qpps_actual: {} / {} = {}".format(m1_enc_diff, time_delta_secs, m1_qpps_actual))
    # print("m2_qpps_actual: {} / {} = {}".format(m2_enc_diff, time_delta_secs, m2_qpps_actual))

    right_angular_v = m1_qpps_actual / float(ticks_per_radian)
    left_angular_v = m2_qpps_actual / float(ticks_per_radian)
    # print("left_angular_v: {}".format(left_angular_v))

    right_linear_v = right_angular_v * float(wheel_radius)
    left_linear_v = left_angular_v * float(wheel_radius)
    # print("right_linear_v: {}".format(left_linear_v))
    # print("left_linear_v: {}".format(left_linear_v))

    x_linear_v = (right_linear_v + left_linear_v) / 2.0
    y_linear_v = 0  # Because the robot is nonholonomic
    z_angular_v = (right_linear_v - left_linear_v) / float(wheel_dist)
    # print("x_linear_v: {}".format(x_linear_v))
    # print("z_angular_v: {}".format(z_angular_v))

    # 2D rotation matrix math https://en.wikipedia.org/wiki/Rotation_matrix
    # But since y_linear_v = 0, we don't actually need the second part of each equation
    world_x_velocity = x_linear_v * cos(world_theta) - y_linear_v * sin(world_theta)
    world_y_velocity = x_linear_v * sin(world_theta) + y_linear_v * cos(world_theta)
    world_angular_velocity = z_angular_v
    # print("world_x_velocity: {}".format(world_x_velocity))
    # print("world_y_velocity: {}".format(world_y_velocity))
    # print("world_angular_velocity: {}".format(world_angular_velocity))

    world_x = world_x + (world_x_velocity * time_delta_secs)
    world_y = world_y + (world_y_velocity * time_delta_secs)
    world_theta = world_theta + (world_angular_velocity * time_delta_secs)
    # print("world coordinates: ({}, {}, {})".format(world_x, world_y, world_theta))

    # Convert world orientation (theta) to a Quaternion for use with tf and Odometry
    quat_vals = tf.transformations.quaternion_from_euler(0, 0, world_theta)
    quat = Quaternion()
    quat.x = quat_vals[0]
    quat.y = quat_vals[1]
    quat.z = quat_vals[2]
    quat.w = quat_vals[3]

    odom = Odometry()
    odom.header.stamp = now_odom_time
    odom.header.frame_id = world_frame_id
    odom.pose.pose.position.x = world_x
    odom.pose.pose.position.y = world_y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = quat
    odom.child_frame_id = base_frame_id
    odom.twist.twist.linear.x = x_linear_v
    odom.twist.twist.linear.y = 0
    odom.twist.twist.angular.z = z_angular_v
    return odom


def yaw_from_odom_message(odom):
    """Converts an Odometry message into an Euler yaw value
    Parameters:
        :param Odometry odom:

    :rtype: float
    """
    return tf.transformations.euler_from_quaternion(
        [
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ])[2]


if __name__ == "__main__":
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    node = BaseNode()
    node.run()
