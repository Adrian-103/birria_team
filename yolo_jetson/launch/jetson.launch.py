from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_jetson',
            executable='deteccion',
            name='deteccion'
        ),
        Node(
            package='yolo_jetson',
            executable='obstaculos',
            name='obstaculos'
        ),
    ])