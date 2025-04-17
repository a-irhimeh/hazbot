from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    zed_pkg = get_package_share_directory('zed_wrapper')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    config_dir = os.path.join(get_package_share_directory('driver'), 'config')  # Adjust if needed

    return LaunchDescription([
        # Motor Controller
        Node(
            package='driver',
            executable='motor_controller',
            name='motor_controller',
            output='screen'
        ),

        # Odometry Node
        Node(
            package='driver',
            executable='odometry',
            name='odometry_node',
            output='screen'
        ),

        # ZED2 Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_pkg, 'launch', 'zed_camera.launch.py')
            ),
            launch_arguments={
                'camera_model': 'zed2',
                'publish_tf': 'true',
                'base_frame': 'base_link'
            }.items()
        ),

        # Static transform between odom and base_footprint (you might not need this if odometry already publishes the TF)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_basefootprint',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
        ),

        # Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': os.path.join(config_dir, 'nav2_params.yaml'),
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        )
    ])
