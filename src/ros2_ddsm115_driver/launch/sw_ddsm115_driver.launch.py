from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from os import path


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_ddsm115_driver')
    rosparams_file = path.join(pkg_share, 'config', 'sw_ddsm115_driver_config.yaml')
    return LaunchDescription([
        Node(
            package='ros2_ddsm115_driver',
            executable='sw_ddsm115_driver',
            name='sw_ddsm115_driver',
            output='screen',
            parameters=[rosparams_file]
        )
    ])