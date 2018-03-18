#!/usr/bin/env python
from __future__ import print_function
import unittest
import math

import rospy

import b2

PKG = 'b2_base'
NAME = 'b2_base_unittest'


class TestBase(unittest.TestCase):

    def __init__(self, *args):
        super(TestBase, self).__init__(*args)

        # Robot parameters
        self.wheel_dist = 0.220
        self.wheel_radius = 0.0325
        self.ticks_per_radian = 48 * 34
        self.base_frame_id = "base_frame"
        self.world_frame_id = "world_frame"
        self.max_drive_secs = 1
        self.max_qpps = 6000

    def test_calc_speed_command(self):

        tests = [
            # (x_linear, z_angular, m1_expected, m2_expected)
            # x is m/sec, and z is radians/sec
            (0.1, 0.0, 799, 799),
            (0.0, 0.1, 88, -88),
            (0.05, 0.5, 839, -40),
            (1.0, 0.0, self.max_qpps, self.max_qpps),
            (0.0, math.pi/4, 690, -690),
            (0.5, 0.0, 3996, 3996),
        ]
        print()

        for x_linear_cmd, z_angular_cmd, m1_expected, m2_expected in tests:
            actual_cmd = b2.calc_create_speed_cmd(
                x_linear_cmd,
                z_angular_cmd,
                self.wheel_dist,
                self.wheel_radius,
                self.ticks_per_radian,
                self.max_drive_secs,
                self.max_qpps
            )
            print()
            print("### Input x_linear: {}, z_angular: {} ###".format(
                x_linear_cmd, z_angular_cmd))
            print("[m1_qpps] Actual: {}, Expected: {}".format(
                actual_cmd.m1_qpps, m1_expected))
            print("[m2_qpps] Actual: {}, Expected: {}".format(
                actual_cmd.m2_qpps, m2_expected))
            print("[max_secs] Actual: {}, Expected: {}".format(
                actual_cmd.max_secs, self.max_drive_secs))

            self.assertEqual(round(actual_cmd.m1_qpps), m1_expected)
            self.assertEqual(round(actual_cmd.m2_qpps), m2_expected)
            self.assertEqual(round(actual_cmd.max_secs), self.max_drive_secs)

    def test_calc_odometry_single(self):

        # world_x, world_y, world_theta, last_odom_time, m1_qpps, m2_qpps, delta_secs
        # new_world_x, new_world_y, new_world_theta, exp_linear_x, exp_linear_y, exp_angular_z
        tests = [
            (0.0, 0.0, 0.0, rospy.Time(0), 1000, 1000, 1,
                0.125, 0.0, 0.0, 0.125, 0.0, 0.0),
            (0.0, 0.0, 0.0, rospy.Time(0), 1000, 500, 1,
                0.094, 0.0, 0.284, 0.094, 0.0, 0.284),
            (0.0, 0.0, 0.0, rospy.Time(0), 3700, 3700, 1,
                0.463, 0.0, 0.0, 0.463, 0.0, 0.0),
            (3.0, 4.0, 1.707, rospy.Time(0), 700, 2800, 0.5,
                2.985, 4.108, 1.110, 0.219, 0.0, -1.194),
            (3.0, 4.0, 1.707, rospy.Time(12000), 700, 2800, 0.5,
                2.985, 4.108, 1.110, 0.219, 0.0, -1.194),
        ]

        for (
            world_x, world_y, world_theta,
            last_odom_time,
            m1_qpps, m2_qpps, delta_secs,
            exp_new_world_x, exp_new_world_y, exp_new_world_theta,
            exp_linear_x, exp_linear_y, exp_angular_z
        ) in tests:
            print()
            print("#### Input: m1: {}, m2: {}, w({}, {}, {}), secs: {}".format(
                m1_qpps, m2_qpps, world_x, world_y, world_theta, delta_secs))
            t2 = last_odom_time + rospy.Duration(secs=delta_secs)

            m1_enc_diff = m1_qpps * delta_secs
            m2_enc_diff = m2_qpps * delta_secs

            odom = b2.calc_create_odometry(
                m1_enc_diff,
                m2_enc_diff,
                self.ticks_per_radian,
                self.wheel_dist,
                self.wheel_radius,
                world_x,
                world_y,
                world_theta,
                last_odom_time,
                self.base_frame_id,
                self.world_frame_id,
                t2
            )
            self._compare_odometry(odom, exp_new_world_x, exp_new_world_y, exp_new_world_theta,
                                   exp_linear_x, exp_linear_y, exp_angular_z)

    def test_calc_odometry_cumulative(self):
        # (linear_x, angular_z, secs,
        #  exp_world_x, exp_world_y, exp_world_theta, exp_linear_x, exp_linear_y, exp_angular_z)
        tests = [
            (0.5, 0.0, 2,                               # Straight for 1 meter
                1.0, 0.0, 0.0, 0.5, 0.0, 0.0),
            (0.0, math.pi/4, 2,                         # Turn 90 degrees
                1.0, 0.0, math.pi/2, 0.0, 0.0, math.pi/4),
            (0.5, 0.0, 2,                               # Straight for 1 meter
                1.0, 1.0, math.pi/2, 0.5, 0.0, 0.0),
            (0.0, math.pi/4, 3,                         # Turn 135 degrees
                1.0, 1.0, -math.pi/4*3, 0.0, 0.0, math.pi/4),
            (0.5, 0.0, 2 * (1 / math.sin(math.pi/4)),   # Straight back to origin
                0.01, 0.01, -math.pi/4*3, 0.5, 0.0, 0.0),  # Not 0,0 because of 0.1 sec odom update
        ]

        world_x = 0.0
        world_y = 0.0
        world_theta = 0.0
        last_odom_time = rospy.Time(0)
        delta_secs = 0.1

        # delta_nsecs = delta_secs * 10**9

        print()

        for (linear_x, angular_z, secs,
             exp_world_x, exp_world_y, exp_world_theta,
             exp_linear_x, exp_linear_y, exp_angular_z) in tests:

            print()
            print("CMD [x:{}, w:{}] for {} secs".format(linear_x, angular_z, secs))

            for i in range(int(secs / delta_secs)):
                t2 = last_odom_time + rospy.Duration(secs=delta_secs)

                cmd = b2.calc_create_speed_cmd(
                    linear_x, angular_z,
                    self.wheel_dist, self.wheel_radius, self.ticks_per_radian,
                    self.max_drive_secs, self.max_qpps
                )

                print("M1 QPPS: {} M2 QPPS: {}".format(cmd.m1_qpps, cmd.m2_qpps))

                m1_enc_diff = cmd.m1_qpps * delta_secs
                m2_enc_diff = cmd.m2_qpps * delta_secs

                odom = b2.calc_create_odometry(
                    m1_enc_diff,
                    m2_enc_diff,
                    self.ticks_per_radian,
                    self.wheel_dist,
                    self.wheel_radius,
                    world_x,
                    world_y,
                    world_theta,
                    last_odom_time,
                    self.base_frame_id,
                    self.world_frame_id,
                    t2
                )

                world_x = odom.pose.pose.position.x
                world_y = odom.pose.pose.position.y
                world_theta = b2.yaw_from_odom_message(odom)
                last_odom_time = t2
                print("x: {}, y: {}, 0: {}, t2: {}".format(
                    world_x, world_y, world_theta, t2))

            self._compare_odometry(
                odom,
                exp_world_x, exp_world_y, exp_world_theta,
                exp_linear_x, exp_linear_y, exp_angular_z
            )

    def _compare_odometry(self, actual_odom, exp_new_world_x, exp_new_world_y, exp_new_world_theta,
                          exp_linear_x, exp_linear_y, exp_angular_z):
        new_world_x = round(actual_odom.pose.pose.position.x, 3)
        new_world_y = round(actual_odom.pose.pose.position.y, 3)

        new_world_theta = b2.yaw_from_odom_message(actual_odom)

        new_world_linear_x = round(actual_odom.twist.twist.linear.x, 3)
        new_world_linear_y = round(actual_odom.twist.twist.linear.y, 3)
        new_world_angular_z = round(actual_odom.twist.twist.angular.z, 3)

        print("Pose Actual: [{}, {}, {}], Expected: [{}, {}, {}]".format(
            new_world_x, new_world_y, new_world_theta,
            exp_new_world_x, exp_new_world_y, exp_new_world_theta))
        print("Velocity Actual: [{}, {}, {}], Expected: [{}, {}, {}]".format(
            new_world_linear_x, new_world_linear_y, new_world_angular_z,
            exp_linear_x, exp_linear_y, exp_angular_z))

        self.assertAlmostEqual(new_world_x, exp_new_world_x, places=2)
        self.assertAlmostEqual(new_world_y, exp_new_world_y, places=2)
        self.assertAlmostEqual(new_world_theta, exp_new_world_theta, places=2)
        self.assertAlmostEqual(new_world_linear_x, exp_linear_x, places=2)
        self.assertAlmostEqual(new_world_linear_y, exp_linear_y, places=2)
        self.assertAlmostEqual(new_world_angular_z, exp_angular_z, places=2)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestBase)
