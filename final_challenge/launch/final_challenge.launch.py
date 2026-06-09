#!/usr/bin/env python3
"""
Launch file — final_challenge (Rubik Pi)
Levanta todos los nodos excepto master_control (correr por separado).

Uso:
    ros2 launch final_challenge final_challenge.launch.py
    ros2 launch final_challenge final_challenge.launch.py debug:=true
    ros2 launch final_challenge final_challenge.launch.py image_topic:=/cam/image_raw
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'final_challenge'


def generate_launch_description():

    pkg_share = get_package_share_directory(PACKAGE)

    # ---- Rutas a los archivos de parámetros ----
    line_follower_params = os.path.join(pkg_share, 'params', 'line_follower_params.yaml')
    line_pid_params      = os.path.join(pkg_share, 'params', 'line_pid_controller_params.yaml')

    # ---- Argumentos desde la línea de comandos ----
    debug_arg       = DeclareLaunchArgument('debug',       default_value='false')
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='/camera/image_raw')

    # ---- Nodos ----

    camera_node = Node(
        package=PACKAGE,
        executable='camera_node',
        name='camera_node',
        output='screen',
    )

    line_follower_node = Node(
        package=PACKAGE,
        executable='line_follower_cv',
        name='line_follower_cv',
        output='screen',
        parameters=[
            line_follower_params,
            {
                'debug':       LaunchConfiguration('debug'),
                'image_topic': LaunchConfiguration('image_topic'),
            },
        ],
    )

    line_pid_node = Node(
        package=PACKAGE,
        executable='line_pid_controller',
        name='line_pid_controller',
        output='screen',
        parameters=[line_pid_params],
    )

    cont_vel_node = Node(
        package=PACKAGE,
        executable='fuzzy_vel',
        name='fuzzy_velocity_node',
        output='screen',
    )

    odometry_node = Node(
        package=PACKAGE,
        executable='odometry_node',
        name='odometry_node',
        output='screen',
    )

    zebra_detector_node = Node(
        package=PACKAGE,
        executable='intersection',
        name='zebra_detector_node',
        output='screen',
    )

    return LaunchDescription([
        debug_arg,
        image_topic_arg,
        camera_node,
        line_follower_node,
        line_pid_node,
        cont_vel_node,
        odometry_node,
        zebra_detector_node,
    ])
