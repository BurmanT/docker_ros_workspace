#!/usr/bin/env python3
"""
Kinova Gen3 Lite launch file.
Starts:
  1. kortex_bringup  — hardware interface / controllers
  2. robot.launch.py — MoveIt move_group + RViz MotionPlanning panel
  3. move_ur3.py     — our MoveItPy service node
"""
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_ip          = LaunchConfiguration("robot_ip").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)

    # ------------------------------------------------------------------ #
    # 1. Hardware interface + controllers                                  #
    # ------------------------------------------------------------------ #
    kortex_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("kortex_bringup"),
                        "launch",
                        "gen3_lite.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip":          robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz":       "false",   # RViz is handled by robot.launch.py below
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # 2. MoveIt move_group + RViz with MotionPlanning panel               #
    # ------------------------------------------------------------------ #
    kinova_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("kinova_gen3_lite_moveit_config"),
                        "launch",
                        "robot.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip":          robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz":       "true",   # RViz is handled by robot.launch.py below
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # 3. Our MoveItPy service node                                        #
    # ------------------------------------------------------------------ #
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3_lite_gen3_lite_2f",
            package_name="kinova_gen3_lite_moveit_config",
        )
        .robot_description(mappings={"use_fake_hardware": use_fake_hardware})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_node = Node(
        package="ur_test",
        executable="move_ur3.py",
        name="moveit_py_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.to_dict(),
            {"planning_pipelines": {"pipeline_names": ["ompl"]}},
            {
                "plan_request_params": {
                    "planning_attempts": 10,
                    "planning_pipeline": "ompl",
                    "planning_time":     5.0,
                    "max_velocity_scaling_factor":     1.0,
                    "max_acceleration_scaling_factor": 1.0,
                }
            },
        ],
    )

    return [kortex_bringup, kinova_moveit, move_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.1.10",
                description="Robot IP address (ignored when use_fake_hardware:=true)",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Simulate the robot — no physical arm required",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )