#!/usr/bin/env python3
"""
navigation_bringup.launch.py

Launches full Nav2 stack for TurtleBot2 with:
- Static TF publishers (base_footprint → base_link → laser)
- cmd_vel relay to TurtleBot2 velocity topic
- Localization (map server + AMCL)
- Navigation (planner + controller)
- RViz with nav config

Usage:
  ros2 launch turtlebot_nav navigation_bringup.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir  = get_package_share_directory('turtlebot_nav')
    nav2_dir = get_package_share_directory('nav2_bringup')

    map_file    = os.path.join(pkg_dir, 'config', 'table_map.yaml')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'nav.rviz')

    return LaunchDescription([

        # Relay /cmd_vel → /commands/velocity (TurtleBot2 uses this topic)
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            arguments=['/cmd_vel_smoothed', '/commands/velocity'],
            output='screen'
        ),

        # Static TF: base_footprint → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_footprint', 'base_link'],
            name='tf_footprint_to_base'
        ),

        # Static TF: base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.30', '0', '0', '0',
                       'base_link', 'laser'],
            name='tf_base_to_laser'
        ),

        # Localization — map server + AMCL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map':          map_file,
                'params_file':  params_file,
                'use_sim_time': 'false',
                'autostart':    'true',
            }.items()
        ),

        # Navigation — planner + controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file':  params_file,
                'use_sim_time': 'false',
                'autostart':    'true',
            }.items()
        ),

        # RViz — wait 5s for Nav2 to initialize first
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen'
                )
            ]
        ),
    ])