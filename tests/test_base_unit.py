#!/usr/bin/env python
from __future__ import print_function
import unittest

import rospy
import tf.transformations

import b2_base.base_node

PKG = 'b2_base'
NAME = 'b2_base_unittest'


class TestBase(unittest.TestCase):

    def __init__(self, *args):
        super(TestBase, self).__init__(*args)

    def test_calc_speed_command(self):
        # Robot parameters
        wheel_dist = 0.220
        wheel_radius = 0.0325
        ticks_per_radian = 48 * 34
        max_drive_secs = 1
        max_qpps = 6000

        tests = [
            # (x_linear, z_angular, m1_expected, m2_expected)
            # x is m/sec, and z is radians/sec
            (0.1, 0.0, 5022, 5022),
            (0.0, 1.0, 5524, -5524),
            (0.05, 0.5, 5273, -251),
            (1.0, 0.0, max_qpps, max_qpps)
        ]

        for x_linear_cmd, z_angular_cmd, m1_expected, m2_expected in tests:
            actual_cmd = b2_base.base_node.calc_create_speed_cmd(
                x_linear_cmd,
                z_angular_cmd,
                wheel_dist,
                wheel_radius,
                ticks_per_radian,
                max_drive_secs,
                max_qpps
            )
            print()
            print("### Input x_linear: {}, z_angular: {} ###".format(
                x_linear_cmd, z_angular_cmd))
            print("[m1_qpps] Actual: {}, Expected: {}".format(
                actual_cmd.m1_qpps, m1_expected))
            print("[m2_qpps] Actual: {}, Expected: {}".format(
                actual_cmd.m2_qpps, m2_expected))
            print("[max_secs] Actual: {}, Expected: {}".format(
                actual_cmd.max_secs, max_drive_secs))

            self.assertEqual(round(actual_cmd.m1_qpps), m1_expected)
            self.assertEqual(round(actual_cmd.m2_qpps), m2_expected)
            self.assertEqual(round(actual_cmd.max_secs), max_drive_secs)

    def test_calc_odometry(self):
        base_node = b2_base.base_node

        # Robot parameters
        wheel_dist = 0.220
        wheel_radius = 0.0325
        ticks_per_radian = 48 * 34
        base_frame_id = "base_frame"
        world_frame_id = "world_frame"

        # world_x, world_y, world_theta, last_odom_time, m1_qpps_actual, m2_qpps_actual, delta_secs
        # new_world_x, new_world_y, new_world_theta, exp_linear_x, exp_linear_y, exp_angular_z
        tests = [
            (0.0, 0.0, 0.0, rospy.Time(0), 1000, 1000, 1,
                0.125, 0.0, 0.0, 0.125, 0.0, 0.0),
            (0.0, 0.0, 0.0, rospy.Time(0), 1000, 500, 1,
                0.09, 0.026, 0.284, 0.094, 0.0, 0.284),
            (3.0, 4.0, 1.707, rospy.Time(12000), 700, 2800, 0.5,
                3.04, 3.898, 1.11, 0.219, 0.0, -1.194),
        ]

        for (
            world_x, world_y, world_theta,
            last_odom_time,
            m1_qpps_actual, m2_qpps_actual, delta_secs,
            exp_new_world_x, exp_new_world_y, exp_new_world_theta,
            exp_linear_x, exp_linear_y, exp_angular_z
        ) in tests:
            print()
            t2 = last_odom_time + rospy.Duration(secs=delta_secs)

            odom = base_node.calc_create_odometry(
                m1_qpps_actual,
                m2_qpps_actual,
                ticks_per_radian,
                wheel_dist,
                wheel_radius,
                world_x,
                world_y,
                world_theta,
                last_odom_time,
                base_frame_id,
                world_frame_id,
                t2
            )
            new_world_x = round(odom.pose.pose.position.x, 3)
            new_world_y = round(odom.pose.pose.position.y, 3)

            quat_msg = odom.pose.pose.orientation
            euler = tf.transformations.euler_from_quaternion(
                [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w])
            new_world_theta = round(euler[2], 3)

            new_world_linear_x = round(odom.twist.twist.linear.x, 3)
            new_world_linear_y = round(odom.twist.twist.linear.y, 3)
            new_world_angular_z = round(odom.twist.twist.angular.z, 3)

            print("Pose Actual: [{}, {}, {}], Expected: [{}, {}, {}]".format(
                new_world_x, new_world_y, new_world_theta,
                exp_new_world_x, exp_new_world_y, exp_new_world_theta))
            print("Velocity Actual: [{}, {}, {}], Expected: [{}, {}, {}]".format(
                new_world_linear_x, new_world_linear_y, new_world_angular_z,
                exp_linear_x, exp_linear_y, exp_angular_z))

            self.assertEqual(new_world_x, exp_new_world_x)
            self.assertEqual(new_world_y, exp_new_world_y)
            self.assertEqual(new_world_theta, exp_new_world_theta)
            self.assertEqual(new_world_linear_x, exp_linear_x)
            self.assertEqual(new_world_linear_y, exp_linear_y)
            self.assertEqual(new_world_angular_z, exp_angular_z)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestBase)
