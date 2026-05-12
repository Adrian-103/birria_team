from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

pkg_share = get_package_share_directory('point_PID_controller')
params_file = os.path.join(pkg_share, 'config', 'params.yaml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='point_PID_controller',
            executable='point_pid',
            name='point_PID_controller',
            parameters=[params_file]
        ),
    ])
