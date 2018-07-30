from __future__ import print_function
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# from b2_logic.pilot_functions import ProximitySensors, heading_from_odometry
from b2_logic.odometry_helpers import heading_from_odometry
from b2_logic.common_functions import add_radians

# Mode enum
MODE_FORWARD = 0
MODE_OBSTACLE_PLAN = 1
MODE_OBSTACLE_TURN = 2


class PilotNode:
    def __init__(self, loophz, fwd_speed, turn_speed,
                 turn_radians, turn_radians_tolerance, cmd_vel_pub):
        """
        Parameters:
            :param int loophz:
            :param float fwd_speed:
            :param float turn_speed:
            :param float turn_radians:
            :param float turn_radians_tolerance:
            :param rospy.Publisher cmd_vel_pub:
        """
        self._loophz = loophz
        self._fwd_speed = fwd_speed
        self._turn_speed = turn_speed
        self._turn_radians = turn_radians
        self._turn_radians_tolerance = turn_radians_tolerance
        self._cmd_vel_pub = cmd_vel_pub

        self._prox_sensor = False
        self._odom = Odometry()
        self._state_lock = threading.RLock()

        self._current_heading = 0.0      # radians
        self._mode = MODE_OBSTACLE_PLAN  # MODE_XXX enum
        self._heading_goal = 0.0         # radians
        self._obstacle_forward = None    # True/False/None
        self._obstacle_right = None      # True/False/None
        self._obstacle_left = None       # True/False/None
        self._reverse_plan = False       # True/False

    def run(self):
        looprate = rospy.Rate(self._loophz)
        try:
            while not rospy.is_shutdown():

                # Update the heading state
                with self._state_lock:
                    self._current_heading = heading_from_odometry(self._odom)

                self._decide()
                looprate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ROSInterruptException received in main loop")

    # ~~~~~~~~~~~~~~~~~~~~~~~~
    #  Subscription callbacks
    # ~~~~~~~~~~~~~~~~~~~~~~~~
    def prox_callback(self, msg):
        """
            :param Proximity msg: The Proximity message
        """
        with self._state_lock:
            self._prox_sensor = msg.sensors[0]

    def odom_callback(self, msg):
        """
            :param Odometry msg: The Odometry message
        """
        with self._state_lock:
            self._odom = msg

    # ~~~~~~~~~~~~~~~~~~~~
    #  Non-public methods
    # ~~~~~~~~~~~~~~~~~~~~
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

    def _set_forward_mode(self):
        self._obstacle_forward = None
        self._obstacle_right = None
        self._obstacle_left = None
        self._reverse_plan = False
        self._mode = MODE_FORWARD

    def _decide(self):
        if self._mode == MODE_FORWARD:
            if self._prox_sensor is True:
                # If driving forward, and center sensor detects obstacle
                # --> stop and enter obstacle mode
                self._send_drive_cmd(0)
                self._mode = MODE_OBSTACLE_PLAN
            else:
                # No obstacle, so command base forward some more
                self._send_drive_cmd(self._fwd_speed)

        else:  # Mode is either _PLAN or _TURN

            # Need to calculate the heading to which to turn next
            if self._mode == MODE_OBSTACLE_PLAN:
                self._process_obstacle_plan()

            # Execute the turn to the target heading
            if self._mode == MODE_OBSTACLE_TURN:
                self._process_obstacle_turn()

    def _process_obstacle_plan(self):
        """
        Note, the logic here assumes that if self._obstacle_XXX is None
        then we haven't yet been in position to test it. Once we test that
        position, we set the value to either True (obstacle) or False (clear)
        then calculate the turn and switch into TURN mode.

        Therefore, if we are in PLAN mode, we can determine which sides we need
        to test still by examiming the self._obstacle_XXX state.

        Example:
           If in PLAN mode and self._obstacle_forward is NONE, we need to
              test the front position, and if TRUE, turn to the right side.
           If in PLAN mode and self._obstacle_forward is TRUE,
              and self._obstacle_right is NONE: we have turned to the right
              but have not yet tested the right side for an obstacle. So test
              the position and if TRUE, we need to turn to the left side.
        """

        if self._obstacle_forward in (None, False):

            if self._prox_sensor is True:
                # Calculate the turn to check the right side
                self._obstacle_forward = True
                self._heading_goal = add_radians(
                    self._current_heading, -self._turn_radians)
                self._mode = MODE_OBSTACLE_TURN
            else:
                self._set_forward_mode()

        elif self._obstacle_right is None:
            if self._prox_sensor is True:
                # Calculate the turn to check the left side
                # We've already turned to the right, so we need to turn 180 to test
                # the left side
                self._obstacle_right = True
                self._heading_goal = add_radians(
                    self._current_heading, self._turn_radians * 2)
                self._mode = MODE_OBSTACLE_TURN
            else:
                self._set_forward_mode()

        elif self._obstacle_left is None:
            if self._prox_sensor is True:
                # All three of fwd, right, left are blocked
                self._obstacle_left = True
                self._heading_goal = add_radians(
                    self._current_heading, self._turn_radians)
                self._mode = MODE_OBSTACLE_TURN
                self._reverse_plan = True
            else:
                self._set_forward_mode()

        elif self._reverse_plan is True:
            # We were performing a turn to reverse. Since we're in plan mode
            # again, this means the turn is complete
            self._set_forward_mode()

        else:
            # # This would be the case where forward, right, and left are blocked
            # # So we need to reverse out, which is turning 90-deg more left
            # self._heading_goal = add_radians(
            #     self._current_heading, self._turn_radians)
            # self._mode = MODE_OBSTACLE_TURN
            # self._reverse_plan = True

            # This should not be possible
            message = "Obstacle plan logic reached else block that should not be possible"
            rospy.logerr(message)
            raise RuntimeError(message)

    def _process_obstacle_turn(self):
        turn_delta = self._heading_goal - self._current_heading
        if abs(turn_delta) > self._turn_radians_tolerance:
            # We still need to turn some more
            if turn_delta < 0.0:
                self._send_turn_cmd(-self._turn_speed)
            else:
                self._send_drive_cmd(self._turn_speed)

        else:
            # We are done turning, back to obstacle planning
            self._mode = MODE_OBSTACLE_PLAN


# class ProximitySensors:
#     def __init__(self):
#         self.left = False
#         self.center = False
#         self.right = False
