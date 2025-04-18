from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dobot_m1',
            executable='m1_controller_node.py',
            name='dobot_m1_controller',
            output='screen',
            parameters=[{
                'base_linear_speed': 20.0,
                'base_angular_speed': 5.0,
                'base_z_speed': 10.0,
                'min_speed_multiplier': 0.5,
                'max_speed_multiplier': 3.0
            }]
        )
    ]) 