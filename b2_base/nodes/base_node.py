#!/usr/bin/env python
from __future__ import print_function

from math import pi as pi

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import tf

from roboclaw_driver.msg import SpeedCommand, Stats
from b2_logic.nodes.base import BaseNode


DEFAULT_NODE_NAME = "base_node"

# Subscribes
DEFAULT_CMD_VEL_TOPIC = "cmd_vel"
DEFAULT_ROBOCLAW_FRONT_STATS_TOPIC = "roboclaw_front/stats"
DEFAULT_ROBOCLAW_REAR_STATS_TOPIC = "roboclaw_rear/stats"

# Publishes
DEFAULT_SPEED_CMD_TOPIC = "roboclaw/speed_command"
DEFAULT_ODOM_TOPIC = "odom"

# Default Parameters
DEFAULT_LOOP_HZ = 10  # hertz
DEFAULT_WHEEL_DIST = 0.180  # meters
DEFAULT_WHEEL_RADIUS = 0.0325  # meters
DEFAULT_WHEEL_SLIP_FACTOR = 0.5  # Decimal % of angular motion lost to slip
DEFAULT_TICKS_PER_ROTATION = 48 * 34
DEFAULT_MAX_QPPS = 3700
DEFAULT_MAX_ACCEL = 20000
DEFAULT_MAX_X_LINEAR_VEL = 0.5      # meters/sec
DEFAULT_MAX_Z_ANGULAR_VEL = pi / 2  # radians/sec
DEFAULT_MAX_DRIVE_SECS = 1
DEFAULT_ODOM_FRAME_ID = "odom"
DEFAULT_PUBLISH_ODOM_TF = True
DEFAULT_BASE_FRAME_ID = "base_link"
DEFAULT_DEADMAN_SECS = 1
DEFAULT_LOG_LEVEL = "info"


def parse_log_level(levelstr):
    map = {
        "debug": rospy.DEBUG,
        "info":  rospy.INFO,
        "warn":  rospy.WARN,
        "error": rospy.ERROR,
        "fatal": rospy.FATAL
    }
    return map.get(levelstr.lower())


if __name__ == "__main__":
    log_level = parse_log_level(rospy.get_param("~log_level", DEFAULT_LOG_LEVEL))
    rospy.init_node(DEFAULT_NODE_NAME, log_level=log_level)
    node_name = rospy.get_name()

    wheel_dist = rospy.get_param("~wheel_dist", DEFAULT_WHEEL_DIST)
    wheel_radius = rospy.get_param("~wheel_radius", DEFAULT_WHEEL_RADIUS)
    wheel_slip_factor = rospy.get_param("~wheel_slip_factor", DEFAULT_WHEEL_SLIP_FACTOR)
    ticks_per_rotation = rospy.get_param("~ticks_per_rotation", DEFAULT_TICKS_PER_ROTATION)
    max_drive_secs = rospy.get_param("~max_drive_secs", DEFAULT_MAX_DRIVE_SECS)
    deadman_secs = rospy.get_param("~deadman_secs", DEFAULT_DEADMAN_SECS)
    max_qpps = rospy.get_param("~max_qpps", DEFAULT_MAX_QPPS)
    max_x_lin_vel = rospy.get_param("~max_x_lin_vel", DEFAULT_MAX_X_LINEAR_VEL)
    max_z_ang_vel = rospy.get_param("~max_z_ang_vel", DEFAULT_MAX_Z_ANGULAR_VEL)
    max_accel = rospy.get_param("~max_accel", DEFAULT_MAX_ACCEL)
    base_frame_id = rospy.get_param("~base_frame_id", DEFAULT_BASE_FRAME_ID)
    world_frame_id = rospy.get_param("~odom_frame_id", DEFAULT_ODOM_FRAME_ID)
    loop_hz = rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ)
    publish_odom_tf = rospy.get_param("~publish_odom_tf", DEFAULT_PUBLISH_ODOM_TF) 

    # Publishes
    speed_cmd_pub = rospy.Publisher(
        rospy.get_param('~speed_cmd_topic', DEFAULT_SPEED_CMD_TOPIC),
        SpeedCommand,
        queue_size=1
    )
    odom_pub = rospy.Publisher(
        rospy.get_param('~odom_topic', DEFAULT_ODOM_TOPIC),
        Odometry,
        queue_size=1
    )
    
    tf_broadcaster = None
    if publish_odom_tf:
        tf_broadcaster = tf.broadcaster.TransformBroadcaster()

    node = BaseNode(wheel_dist, wheel_radius, wheel_slip_factor, ticks_per_rotation,
                    max_drive_secs, deadman_secs,
                    max_qpps, max_x_lin_vel, max_z_ang_vel, max_accel,
                    base_frame_id, world_frame_id,
                    speed_cmd_pub, odom_pub, publish_odom_tf, tf_broadcaster)

    # Twist message Subscriber
    rospy.Subscriber(
        rospy.get_param("~cmd_vel_topic", DEFAULT_CMD_VEL_TOPIC),
        Twist,
        node.cmd_vel_callback
    )

    # Roboclaw Stats message Subscriber
    rospy.Subscriber(
        rospy.get_param("~roboclaw_front_stats_topic", DEFAULT_ROBOCLAW_FRONT_STATS_TOPIC),
        Stats,
        node.roboclaw_stats_callback,
        "front"
    )

    rospy.Subscriber(
        rospy.get_param("~roboclaw_rear_stats_topic", DEFAULT_ROBOCLAW_REAR_STATS_TOPIC),
        Stats,
        node.roboclaw_stats_callback,
        "rear"
    )

    node.run(loop_hz)
