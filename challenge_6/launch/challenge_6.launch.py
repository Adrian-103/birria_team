from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    debug_arg = DeclareLaunchArgument('debug', default_value='false')

    return LaunchDescription([
        debug_arg,
        Node(
            package='challenge_6',
            executable='camera_node',
            name='camera_node'
        ),
        Node(
            package='challenge_6',
            executable='hough_node',
            name='hough_node',
            parameters=[{'debug': LaunchConfiguration('debug')}]
        ),
        Node(
            package='challenge_6',
            executable='vision',
            name='vision'
        ),
        Node(
            package='challenge_6',
            executable='odometry_node',
            name='odometry_node'
        ),
        Node(
            package='challenge_6',
            executable='pid_challenge6',
            name='pid_challenge6'
        ),
    ])