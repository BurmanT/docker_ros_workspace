#!/usr/bin/env python3
"""
joy_teleop.launch.py

Starts:
  1. joy_node      — reads Xbox controller
  2. joy_teleop.py — moves Kinova arm + toggles recording

Usage:
  ros2 launch ur_test joy_teleop.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "device_id":       0,
            "autorepeat_rate": 0.0,
            "deadzone":        0.05,
        }],
    )

    teleop_node = Node(
        package="ur_test",
        executable="joy_teleop.py",
        name="joy_teleop",
        output="screen",
    )

    return LaunchDescription([joy_node, teleop_node])