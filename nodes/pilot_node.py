#!/usr/bin/env python
from __future__ import print_function
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from b2_logic.nodes.pilot import PilotNode
from b2.msg import Proximity


DEFAULT_NODE_NAME = "pilot_node"

# Subscribes
DEFAULT_PROXIMITY_TOPIC = "ir_sensors/proximity"
DEFAULT_ODOMETRY_TOPIC = "base_node/odom"

# Publishes
DEFAULT_CMD_TOPIC = "base_node/cmd_vel"

DEFAULT_LOOP_HZ = 2                # hertz
DEFAULT_FWD_SPEED = 0.5            # m/sec
DEFAULT_TURN_SPEED = math.pi / 4   # radians/sec
DEFAULT_TURN_DEGREES = 135         # degrees, will be converted to radians
DEFAULT_TURN_DEGREE_TOLERANCE = 5  # degrees, will be converted to radians


if __name__ == "__main__":
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)

    node_name = rospy.get_name()
    loophz = rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ)
    fwd_speed = rospy.get_param("~fwd_speed", DEFAULT_FWD_SPEED)
    turn_speed = rospy.get_param("~turn_speed", DEFAULT_TURN_SPEED)
    turn_radians = math.radians(rospy.get_param("~turn_degrees", DEFAULT_TURN_DEGREES))
    turn_radians_tolerance = math.radians(
        rospy.get_param("~turn_degree_tolerance", DEFAULT_TURN_DEGREE_TOLERANCE))

    cmd_vel_pub = rospy.Publisher(
        rospy.get_param('~cmd_topic', DEFAULT_CMD_TOPIC),
        Twist,
        queue_size=1
    )

    node = PilotNode(
        loophz, fwd_speed,
        turn_speed, turn_radians,
        turn_radians_tolerance, cmd_vel_pub
    )

    rospy.Subscriber(
        rospy.get_param("~proximity_topic", DEFAULT_PROXIMITY_TOPIC),
        Proximity,
        node.cmd_prox_callback
    )

    rospy.Subscriber(
        rospy.get_param("~odom_topic", DEFAULT_ODOMETRY_TOPIC),
        Odometry,
        node.odom_callback
    )

    node.run()
