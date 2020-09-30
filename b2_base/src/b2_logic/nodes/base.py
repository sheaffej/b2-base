from __future__ import print_function
import threading
import rospy

from roboclaw_driver.msg import Stats
from b2_logic.odometry_helpers import yaw_from_odom_message
from b2_logic.base_functions import (
    calc_create_speed_cmd,
    calc_base_frame_velocity_from_encoder_diffs,
    calc_odometry_from_base_velocity
)


class BaseNode:
    def __init__(self, wheel_dist, wheel_radius, wheel_slip_factor, ticks_per_rotation,
                 max_drive_secs, deadman_secs, 
                 max_qpps, max_x_lin_vel, max_z_ang_vel, max_accel,
                 base_frame_id, world_frame_id,
                 speed_cmd_pub, odom_pub, publish_odom_tf, tf_broadcaster):

        self._wheel_dist = wheel_dist
        self._wheel_radius = wheel_radius
        self._wheel_slip_factor = wheel_slip_factor
        self._ticks_per_rotation = ticks_per_rotation
        self._max_drive_secs = max_drive_secs
        self._deadman_secs = deadman_secs
        self._max_qpps = max_qpps
        self._max_x_lin_vel = max_x_lin_vel
        self._max_z_ang_vel = max_z_ang_vel
        self._max_accel = max_accel
        self._base_frame_id = base_frame_id
        self._world_frame_id = world_frame_id

        self._speed_cmd_pub = speed_cmd_pub
        self._odom_pub = odom_pub
        self._publish_odom_tf = publish_odom_tf
        self._tf_broadcaster = tf_broadcaster

        # Init Twist command state
        self._x_linear_cmd = 0.0
        self._z_angular_cmd = 0.0

        # Last time we received a Twist message from the Pilot
        # If we don't get a message after deadman_secs, we stop the base
        self._last_cmd_vel_time = rospy.Time()  # type: rospy.Time

        # Init Odometry state
        self._world_x = 0.0
        self._world_y = 0.0
        self._world_theta = 0.0
        self._last_odom_time = None   # type: rospy.Time

        # Init Roboclaw stats state
        self._roboclaw_front_stats = None  # type: Stats
        # self._roboclaw_rear_stats = None  # type: Stats

        # Roboclaw encoder state
        self._m1_front_enc_prev = 0
        self._m2_front_enc_prev = 0
        # self._m1_rear_enc_prev = 0
        # self._m2_rear_enc_prev = 0

        self._stats_lock = threading.RLock()  # To serialize access to the qpps stats
        self._cmd_vel_lock = threading.RLock()  # To serialize access to x/z command variables

    def run(self, loop_hz):
        """Runs the main loop of the node.
        Sends motor commands, and publishes odometry.
        """
        rospy.logdebug("Running node")
        looprate = rospy.Rate(loop_hz)

        # Set initial states
        if self._roboclaw_front_stats is not None:
            self._m1_front_enc_prev = self._roboclaw_front_stats.m1_enc_val
            self._m2_front_enc_prev = self._roboclaw_front_stats.m2_enc_val
        # if self._roboclaw_rear_stats is not None:
        #     self._m1_rear_enc_prev = self._roboclaw_rear_stats.m1_enc_val
        #     self._m2_rear_enc_prev = self._roboclaw_rear_stats.m2_enc_val
        self._last_odom_time = rospy.get_rostime()
        self._last_cmd_vel_time = rospy.get_rostime()

        try:
            while not rospy.is_shutdown():
                self.process_base_loop()
                looprate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ROSInterruptException received in main loop")

    def cmd_vel_callback(self, msg):
        """Called by the Twist cmd_vel message subscriber.

        Parameters:
            :param Twist msg: Twist command velocity message
        """
        with self._cmd_vel_lock:

            if ((self._last_cmd_vel_time - rospy.get_rostime()).to_sec() < 1.0
                    and msg.linear.x == self._x_linear_cmd
                    and msg.angular.z == self._z_angular_cmd):
                rospy.logdebug("Cmd Vel received, but no change in values")

            else:
                self._x_linear_cmd = msg.linear.x
                self._z_angular_cmd = msg.angular.z
                self._last_cmd_vel_time = rospy.get_rostime()
                rospy.logdebug("CMD Vel - X: {}  |  Z: {}".format(msg.linear.x, msg.angular.z))

    def roboclaw_stats_callback(self, stats, callback_args):
        """Called by the Roboclaw Stats message subscriber

        Parameters:
            :param Stats stats: Roboclaw Stats message
            :parmm list callback_args: Arguments to this function (i.e. "front" or "rear)
        """
        with self._stats_lock:
            if "front" in callback_args:
                self._roboclaw_front_stats = stats
            elif "rear" in callback_args:
                self._roboclaw_rear_stats = stats
            else:
                rospy.logwarn("roboclaw_stats_callback: Unsure which stats to read")
                rospy.logwarn("callback_args: {}".format(callback_args))

    def process_base_loop(self):
        # If the last command was over 1 sec ago, stop the base
        if (
            self._last_cmd_vel_time is None or
            (rospy.get_rostime() - self._last_cmd_vel_time).to_sec() > self._deadman_secs
        ):
            self._x_linear_cmd = 0.0
            self._z_angular_cmd = 0.0

        # ---------------------------------
        # Calculate and send motor commands
        # ---------------------------------
        with self._cmd_vel_lock:
            x_linear_cmd = self._x_linear_cmd
            z_angular_cmd = self._z_angular_cmd

        # Clamp the velocities to the max configured for the base
        x_linear_cmd = max(-self._max_x_lin_vel, min(x_linear_cmd, self._max_x_lin_vel))
        z_angular_cmd = max(-self._max_z_ang_vel, min(z_angular_cmd, self._max_z_ang_vel))

        cmd = calc_create_speed_cmd(
            x_linear_cmd, z_angular_cmd,
            self._wheel_dist, self._wheel_radius, self._wheel_slip_factor,
            self._ticks_per_rotation, self._max_drive_secs, self._max_qpps, self._max_accel
        )
        self._speed_cmd_pub.publish(cmd)

        # -------------------------------
        # Calculate and publish Odometry
        # -------------------------------

        # if self._roboclaw_front_stats is None or self._roboclaw_rear_stats is None:
        if self._roboclaw_front_stats is None:
            rospy.loginfo("Insufficient roboclaw stats received, skipping odometry calculation")
            return

        with self._stats_lock:
            # Calculate change in encoder readings
            m1_front_enc_diff = self._roboclaw_front_stats.m1_enc_val - self._m1_front_enc_prev
            m2_front_enc_diff = self._roboclaw_front_stats.m2_enc_val - self._m2_front_enc_prev
            # m1_rear_enc_diff = self._roboclaw_rear_stats.m1_enc_val - self._m1_rear_enc_prev
            # m2_rear_enc_diff = self._roboclaw_rear_stats.m2_enc_val - self._m2_rear_enc_prev

            self._m1_front_enc_prev = self._roboclaw_front_stats.m1_enc_val
            self._m2_front_enc_prev = self._roboclaw_front_stats.m2_enc_val
            # self._m1_rear_enc_prev = self._roboclaw_rear_stats.m1_enc_val
            # self._m2_rear_enc_prev = self._roboclaw_rear_stats.m2_enc_val

            # Since we have a two Roboclaw robot, take the average of the encoder diffs
            # from each Roboclaw for each side.
            # m1_enc_diff = (m1_front_enc_diff + m1_rear_enc_diff) / 2
            # m2_enc_diff = (m2_front_enc_diff + m2_rear_enc_diff) / 2
            m1_enc_diff = m1_front_enc_diff
            m2_enc_diff = m2_front_enc_diff

            # We take the nowtime from the Stats message so it matches the encoder values.
            # Otherwise we would get timing variances based on when the loop runs compared to
            # when the stats were measured..
            # Since we have a two Roboclaw robot, take the latest stats timestamp from either
            # Roboclaw.
            front_stamp = self._roboclaw_front_stats.header.stamp
            # rear_stamp = self._roboclaw_rear_stats.header.stamp
            # nowtime = max(front_stamp, rear_stamp)
            nowtime = front_stamp

        x_linear_v, y_linear_v, z_angular_v = calc_base_frame_velocity_from_encoder_diffs(
            m1_enc_diff, m2_enc_diff, self._ticks_per_rotation,
            self._wheel_radius, self._wheel_dist, self._wheel_slip_factor,
            self._last_odom_time, nowtime
        )

        time_delta_secs = (nowtime - self._last_odom_time).to_sec()
        self._last_odom_time = nowtime

        odom = calc_odometry_from_base_velocity(
            x_linear_v, y_linear_v, z_angular_v,
            self._world_x, self._world_y, self._world_theta,
            time_delta_secs, nowtime,
            self._base_frame_id, self._world_frame_id
        )
        self._odom_pub.publish(odom)

        # -----------------------------------------
        # Calculate and broacast tf transformation
        # -----------------------------------------
        if self._publish_odom_tf:
            self._world_x = odom.pose.pose.position.x
            self._world_y = odom.pose.pose.position.y
            self._world_theta = yaw_from_odom_message(odom)
            quat = odom.pose.pose.orientation

            self._tf_broadcaster.sendTransform(
                (self._world_x, self._world_y, 0),
                (quat.x, quat.y, quat.z, quat.w),
                nowtime,
                self._base_frame_id,
                self._world_frame_id
            )

        self._last_odom_time = nowtime

        rospy.logdebug(
            "World position: [{}, {}] heading: {}".format(
                self._world_x, self._world_y, self._world_theta))
        rospy.logdebug(
            "Forward speed: {}, Turn speed: {}".format(
                self._x_linear_cmd, self._z_angular_cmd))
