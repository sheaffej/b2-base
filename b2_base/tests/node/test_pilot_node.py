#!/usr/bin/env python
import unittest
import threading

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from b2_base.msg import Proximity
from b2_logic.odometry_helpers import heading_from_odometry

PKG = 'b2_base'
NAME = 'pilot_node_test'

DEFAULT_SENSOR_TOPIC = "/ir_sensors/proximity"
DEFAULT_ODOM_TOPIC = "/base_node/odom"
DEFAULT_CMD_VEL_TOPIC = "/base_node/cmd_vel"


class TestPilotNode(unittest.TestCase):
    def setUp(self):
        print()
        rospy.init_node(NAME, log_level=rospy.DEBUG, anonymous=True)
        self.state_lock = threading.RLock()
        self.cmd_vel = Twist()

        self.max_linear_x = rospy.get_param("pilot_node/max_fwd_speed")
        self.max_angular_z = rospy.get_param("pilot_node/max_turn_speed")

        rospy.Subscriber(
            rospy.get_param("~cmd_vel_topic", DEFAULT_CMD_VEL_TOPIC),
            Twist,
            self._cmd_vel_callback
        )

        self.pub_proximity = rospy.Publisher(
            rospy.get_param("~proximity_topic", DEFAULT_SENSOR_TOPIC),
            Proximity,
            queue_size=1
        )

        self.pub_odom = rospy.Publisher(
            rospy.get_param("~odom_topic", DEFAULT_ODOM_TOPIC),
            Odometry,
            queue_size=1
        )

        rospy.sleep(1)  # Let subscribers connect

    def test_fwd_obstacle(self):
        # Verify driving forward
        rospy.sleep(1)
        self._verify_cmd_vel(self.max_linear_x, 0.0)

        # Introduce obstacle
        rospy.sleep(2)
        self.pub_proximity.publish(Proximity(sensors=[True]))
        rospy.sleep(1)

        # Verify turning to the right
        self._verify_cmd_vel(0.0, -1.0, z_delta=0.5)

        # Remove obstacle
        self.pub_proximity.publish(Proximity(sensors=[False]))
        rospy.sleep(2)

        # Verify driving forward
        self._verify_cmd_vel(self.max_linear_x, 0.0)

    def _cmd_vel_callback(self, msg):
        with self.state_lock:
            self.cmd_vel = msg

    def _get_heading_velocity(self):
        with self.state_lock:
            heading = heading_from_odometry(self.odom)
            linear_x = self.odom.twist.twist.linear.x
            print("   Heading: {}".format(heading))
            print("   X velocity: {}".format(linear_x))
        return heading, linear_x

    def _verify_cmd_vel(self, exp_linear_x, exp_angular_z, x_delta=0.0, z_delta=0.0):
        with self.state_lock:
            print(" Linear X is {}, expected is {}".format(self.cmd_vel.linear.x, exp_linear_x))
            self.assertAlmostEquals(self.cmd_vel.linear.x, exp_linear_x, delta=x_delta)
            print(" Angular Z is {}, expected is {}".format(self.cmd_vel.angular.z, exp_angular_z))
            self.assertAlmostEquals(self.cmd_vel.angular.z, exp_angular_z, delta=z_delta)
            print("PASS")


if __name__ == '__main__':

    import rostest
    rostest.rosrun(PKG, NAME, TestPilotNode)
