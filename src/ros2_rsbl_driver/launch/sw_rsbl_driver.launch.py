from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_rsbl_driver')
    rosparams_file = path.join(pkg_share, 'config', 'sw_rsbl_driver_config.yaml')
    return LaunchDescription([
        Node(
            package='ros2_rsbl_driver',
            executable='sw_rsbl_driver',
            name='sw_rsbl_driver',
            output='screen',
            parameters=[rosparams_file]
        )
    ])