"""Minimal launch sketch for the ROS2 integration boundary.

The root replay is intentionally dependency-free; this file is a template for a
ROS2 workspace where Gazebo and Nav2 are installed.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="nav2_bringup", executable="bringup_launch.py",
             name="nav2", output="screen",
             parameters=["config/nav2_params.yaml"]),
    ])
