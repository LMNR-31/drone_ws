"""
drone_controller.launch.py
Launch file for my_drone_controller/drone_node.

Usage:
  ros2 launch my_drone_controller drone_controller.launch.py
  ros2 launch my_drone_controller drone_controller.launch.py hover_altitude:=15.0
  ros2 launch my_drone_controller drone_controller.launch.py \
      hover_altitude:=10.0 log_level:=debug

All parameters override the corresponding values from drone_config.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("my_drone_controller")
    default_config = os.path.join(pkg_share, "config", "drone_config.yaml")

    # ── Declare CLI arguments ────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_config,
            description="Full path to the drone_config.yaml parameter file",
        ),
        DeclareLaunchArgument(
            "hover_altitude",
            default_value="",
            description="Override hover_altitude [m] (empty = use params_file value)",
        ),
        DeclareLaunchArgument(
            "max_altitude",
            default_value="",
            description="Override max_altitude [m] (empty = use params_file value)",
        ),
        DeclareLaunchArgument(
            "waypoint_duration",
            default_value="",
            description="Override waypoint_duration [s] (empty = use params_file value)",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS 2 log level: debug | info | warn | error",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock (true for Gazebo, false for hardware)",
        ),
    ]

    # ── Node definition ──────────────────────────────────────────────────────
    drone_node = Node(
        package="my_drone_controller",
        executable="drone_node",
        name="drone_controller_completo",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=[
            "--ros-args",
            "--log-level", LaunchConfiguration("log_level"),
        ],
    )

    return LaunchDescription(
        args
        + [
            LogInfo(msg=["Launching drone_node with config: ", LaunchConfiguration("params_file")]),
            drone_node,
        ]
    )
