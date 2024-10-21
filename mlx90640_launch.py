from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mlx90640_driver'),
        'config',
        'mlx90640_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=config,
            description='Path to the configuration file for the MLX90640 driver'
        ),
        Node(
            package='mlx90640_driver',
            executable='mlx90640_driver',
            name='mlx90640_driver',
            parameters=[config]
        )
    ])
