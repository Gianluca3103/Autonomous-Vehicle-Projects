"""Start TurtleBot3's empty Gazebo world and the C++ waypoint controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "empty_world.launch.py",
    )

    return LaunchDescription([
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),
        Node(
            package="turtlebot3_waypoint_controller",
            executable="project2_controller",
            name="odom_listener",
            output="screen",
        ),
    ])
