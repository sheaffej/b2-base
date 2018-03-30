#!/usr/bin/env python
from __future__ import print_function
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import tf

from roboclaw_driver.msg import SpeedCommand, Stats
import b2

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
                cmd = b2.calc_create_speed_cmd(
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

                odom = b2.calc_create_odometry(
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
