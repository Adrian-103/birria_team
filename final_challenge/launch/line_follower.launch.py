#!/usr/bin/env python3
"""Launch the CV line follower with the YAML parameter file.

    ros2 launch <your_pkg> line_follower.launch.py
    ros2 launch <your_pkg> line_follower.launch.py debug:=false image_topic:=/cam/image
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# >>> change this to your package name <<<
PACKAGE_NAME = "final_challenge"


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "params", "line_follower_params.yaml")

    debug_arg = DeclareLaunchArgument("debug", default_value="true")
    image_arg = DeclareLaunchArgument("image_topic", default_value="/camera/image_raw")

    node = Node(
        package=PACKAGE_NAME,
        executable="line_follower_cv",
        name="line_follower_cv",
        output="screen",
        parameters=[
            params_file,
            {   # CLI overrides win over the YAML file
                "debug": LaunchConfiguration("debug"),
                "image_topic": LaunchConfiguration("image_topic"),
            },
        ],
    )
    
    pid_node = Node(
        package=PACKAGE_NAME,
        executable='pid_test_node',
        name='line_follower_pid',
        output='screen',
    )

    return LaunchDescription([debug_arg, image_arg, node])
