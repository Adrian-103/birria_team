from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='midterm_challenge',
            executable='vision_exe',
            name='vision'
        ),
        Node(
            package='midterm_challenge',
            executable='pid_exe',
            name='point_pid_controller'
        ),
    ])
