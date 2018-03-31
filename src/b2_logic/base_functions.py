from math import pi, sin, cos

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import tf.transformations

from roboclaw_driver.msg import SpeedCommand


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

    ticks_per_radian = ticks_per_rotation / (pi * 2)
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
