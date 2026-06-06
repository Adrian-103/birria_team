from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Nodo de Odometría
        Node(
            package='A_star',
            executable='odometry',
            name='odometry_node',
            output='screen'
        ),
        
        # 2. Nodo Planificador Global
        Node(
            package='A_star',
            executable='global_planner',
            name='global_planner_node',
            output='screen'
        ),
        
        # 3. Nodo Seguidor de Ruta
        Node(
            package='A_star',
            executable='path_follower',
            name='path_follower_node',
            output='screen'
        )
    ])