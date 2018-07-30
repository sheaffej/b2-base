import tf.transformations


def heading_from_odometry(odom):
    """ Extracts the heading (i.e. yaw) from an Odometry message

    Parameters:
        :param Odometry odom: The Odometry message

    Returns: The heading (yaw)
        :rtype: float
    """
    return tf.transformations.euler_from_quaternion(odom.pose.pose.orientation)[2]


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
