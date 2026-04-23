#!/usr/bin/env python3
"""
record_demonstration.launch.py

Starts:
  1. joy_node              — reads Xbox controller
  2. demonstration_recorder — toggles rosbag on RB button press

Usage:
  ros2 launch ur_test record_demonstration.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                "device_id":       0,
                "autorepeat_rate": 0.0,   # only publish on actual input
                "deadzone":        0.05,
            }
        ],
    )

    recorder_node = Node(
        package="ur_test",
        executable="demonstration_recorder.py",
        name="demonstration_recorder",
        output="screen",
    )

    return LaunchDescription([joy_node, recorder_node])