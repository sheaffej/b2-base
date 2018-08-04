from __future__ import print_function
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import tf

from roboclaw_driver.msg import SpeedCommand, Stats
from b2_logic.odometry_helpers import yaw_from_odom_message
from b2_logic.base_functions import calc_create_speed_cmd, calc_create_odometry


class BaseNode:
    def __init__(self, wheel_dist, wheel_radius, ticks_per_rotation,
                 max_drive_secs, max_qpps, base_frame_id, world_frame_id,
                 deadman_secs, speed_cmd_pub, odom_pub, tf_broadcaster):

        self._wheel_dist = wheel_dist
        self._wheel_radius = wheel_radius
        self._ticks_per_rotation = ticks_per_rotation
        self._max_drive_secs = max_drive_secs
        self._max_qpps = max_qpps
        self._base_frame_id = base_frame_id
        self._world_frame_id = world_frame_id
        self._deadman_secs = deadman_secs

        self._speed_cmd_pub = speed_cmd_pub
        self._odom_pub = odom_pub
        self._tf_broadcaster = tf_broadcaster

        # Init Twist command state
        self._x_linear_cmd = 0.0
        self._z_angular_cmd = 0.0
        self._last_cmd_vel_time = rospy.get_rostime()

        # Init Odometry state
        self._world_x = 0.0
        self._world_y = 0.0
        self._world_theta = 0.0
        self._roboclaw_stats = Stats()

        self._stats_lock = threading.RLock()  # To serialize access to the qpps stats
        self._cmd_vel_lock = threading.RLock()  # To serialize access to x/z command variables

    def run(self, loop_hz):
        """Runs the main loop of the node.
        Sends motor commands, and publishes odometry.
        """
        rospy.loginfo("Running node")
        looprate = rospy.Rate(loop_hz)

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

    def cmd_vel_callback(self, msg):
        """Called by the Twist cmd_vel message subscriber.

        Parameters:
            :param Twist msg: Twist command velocity message
        """
        with self._cmd_vel_lock:
            self._x_linear_cmd = msg.linear.x
            self._z_angular_cmd = msg.angular.z
            self._last_cmd_vel_time = rospy.get_rostime()

    def roboclaw_stats_callback(self, stats):
        """Called by the Roboclaw Stats message subscriber

        Parameters:
            :param Stats stats: Roboclaw Stats message
        """
        with self._stats_lock:
            self._roboclaw_stats = stats
