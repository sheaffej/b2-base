from math import pi
from numpy import sign

import tf.transformations
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


def heading_from_odometry(odom):
    return yaw_from_odom_message(odom)


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


def create_odometry_message(world_x, world_y, world_theta,
                            x_linear_v, z_angular_v, odom_time,
                            base_frame_id, world_frame_id):
    # Convert world orientation (theta) to a Quaternion for use with tf and Odometry
    quat_vals = tf.transformations.quaternion_from_euler(0, 0, world_theta)
    quat = Quaternion()
    quat.x = quat_vals[0]
    quat.y = quat_vals[1]
    quat.z = quat_vals[2]
    quat.w = quat_vals[3]

    odom = Odometry()
    odom.header.stamp = odom_time
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


def calc_world_frame_pose(world_x_velocity, world_y_velocity, world_angular_velocity,
                          begin_world_x, begin_world_y, begin_world_theta, time_delta_secs):
    """Given world velocity vectors, movement duration, and beginning world coordinates
    calculate the new world coordinates.
    """
    new_world_x = begin_world_x + (world_x_velocity * time_delta_secs)
    new_world_y = begin_world_y + (world_y_velocity * time_delta_secs)
    new_world_theta = begin_world_theta + (world_angular_velocity * time_delta_secs)
    new_world_theta = normalize_theta(new_world_theta)
    return (new_world_x, new_world_y, new_world_theta)


def normalize_theta(theta):
    """Convert the result of adding or subtracting angular movements to a theta
    that falls within the range of 0.0 >= theta => 2pi, where theta is always positive.
    """
    norm_theta = theta % (pi * 2)
    if norm_theta < 0.0:
        norm_theta = (pi * 2) + norm_theta
    return norm_theta
