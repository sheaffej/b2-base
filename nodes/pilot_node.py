#!/usr/bin/env python
from __future__ import print_function
import threading
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import b2_logic
from b2.msg import Proximity


DEFAULT_NODE_NAME = "pilot_node"

# Subscribes
DEFAULT_PROXIMITY_TOPIC = "ir_sensors/proximity"
DEFAULT_ODOMETRY_TOPIC = "base_node/odom"

# Publishes
DEFAULT_CMD_TOPIC = "base_node/cmd_vel"

DEFAULT_LOOP_HZ = 2  # hertz
DEFAULT_FWD_SPEED = 0.5  # m/sec
DEFAULT_TURN_SPEED = math.pi / 4  # radians/sec
DEFAULT_TURN_DEGREES = 135
DEFAULT_TURN_DEGREE_TOLERANCE = 5

# Mode enum
MODE_FORWARD = 0
MODE_OBSTACLE_PLAN = 1
MODE_OBSTACLE_TURN = 2
MODE_OBSTACLE_REVERSE = 3


class PilotNode:
    def __init__(self):

        self._loophz = rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ)

        self._prox_sensors = b2_logic.ProximitySensors()
        self._odom = Odometry()
        self._state_lock = threading.RLock()

        self._mode = 0
        self._heading_goal = 0.0  # radians

        self._fwd_speed = rospy.get_param("~fwd_speed", DEFAULT_FWD_SPEED)
        self._turn_speed = rospy.get_param("~turn_speed", DEFAULT_TURN_SPEED)
        self._turn_radians = math.radians(rospy.get_param("~turn_degrees", DEFAULT_TURN_DEGREES))
        self._turn_radians_tolerance = math.radians(
            rospy.get_param("~turn_degree_tolerance", DEFAULT_TURN_DEGREE_TOLERANCE))

        # Set up the Proximity message subscriber
        rospy.Subscriber(
            rospy.get_param("~proximity_topic", DEFAULT_PROXIMITY_TOPIC),
            Proximity,
            self._cmd_prox_callback
        )

        # Set up the Odometry message subscriber
        rospy.Subscriber(
            rospy.get_param("~odom_topic", DEFAULT_ODOMETRY_TOPIC),
            Odometry,
            self._odom_callback
        )

        # Set up the publishers
        self._cmd_vel_pub = rospy.Publisher(
            rospy.get_param('~cmd_topic', DEFAULT_CMD_TOPIC),
            Twist,
            queue_size=1
        )

    def run(self):
        looprate = rospy.Rate(self._loophz)
        try:
            while not rospy.is_shutdown():

                if self._mode == MODE_FORWARD and self._prox_sensors.center:
                    self._send_drive_cmd(0)
                    self._mode = MODE_OBSTACLE_PLAN

                with self._state_lock:
                    current_heading = b2_logic.heading_from_odometry(self._odom)

                if self._mode == MODE_FORWARD:
                    self._send_drive_cmd(self._fwd_speed)

                elif self._mode == MODE_OBSTACLE_PLAN:
                    if not self._prox_sensors.right:
                        # Right is clear --> Turn right X degrees
                        self._heading_goal = b2_logic.add_radians(
                            current_heading, -self._turn_radians)
                        self._mode = MODE_OBSTACLE_TURN

                    elif not self._prox_sensors.left:
                        # Left is clear --> Turn left X degrees
                        self._heading_goal = b2_logic.add_radians(
                            current_heading, self._turn_radians)
                        self._mode = MODE_OBSTACLE_TURN

                    else:
                        # Else the front, and both sides are blocked
                        # We have to reverse out of the position
                        self._mode = MODE_OBSTACLE_REVERSE

                elif self._mode == MODE_OBSTACLE_TURN:
                    turn_delta = self._heading_goal - current_heading
                    if abs(turn_delta) > self._turn_radians_tolerance:
                        # We still need to turn some more
                        if turn_delta < 0.0:
                            self._send_turn_cmd(-self._turn_speed)
                        else:
                            self._send_drive_cmd(self._turn_speed)

                    else:
                        # We are done turning
                        self._mode = MODE_FORWARD

                elif self._mode == MODE_OBSTACLE_REVERSE:
                    # Reverse until a side is clear
                    # Then turn X degrees to that side

                    if self._prox_sensors.left and self._prox_sensors.right:
                        # Right & left still blocked --> reverse
                        self._send_drive_cmd(-self._fwd_speed)

                    elif not self._prox_sensors.left:
                        # Left is not blocked --> turn left
                        pass

                looprate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ROSInterruptException received in main loop")

    def _cmd_prox_callback(self, msg):
        """
            :param Proximity msg: The Proximity message
        """
        with self._state_lock:
            cur = b2_logic.ProximitySensors()
            cur.left = msg.sensors[0]
            cur.center = msg.sensors[1]
            cur.right = msg.sensors[2]
            self._prox_sensors = cur

    def _odom_callback(self, msg):
        """
            :param Odometry msg: The Odometry message
        """
        with self._state_lock:
            self._odom = msg

    def _send_drive_cmd(self, speed):
        """ Sends the Twist command for linear.x speed in meters/sec
            :param float speed: Speed in meters/sec for linear.x
        """
        cmd = Twist()
        cmd.linear.x = speed
        self._cmd_vel_pub.publish(cmd)

    def _send_turn_cmd(self, radians_sec):
        """ Sends the Twist command for angular.z speed in radians/sec
        :param float radians_sec: Angular speed in radians/sec for angular.z
        """
        cmd = Twist()
        cmd.angular.z = radians_sec
        self._cmd_vel_pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    node = PilotNode()
    node.run()
