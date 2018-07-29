from nav_msgs.msg import Odometry
import tf.transformations


class ProximitySensors:
    def __init__(self):
        self.left = False
        self.center = False
        self.right = False


def heading_from_odometry(odom):
    """ Extracts the heading (i.e. yaw) from an Odometry message

    Parameters:
        :param Odometry odom: The Odometry message

    Returns: The heading (yaw)
        :rtype: float
    """
    return tf.transformations.euler_from_quaternion(odom.pose.pose.orientation)[2]
