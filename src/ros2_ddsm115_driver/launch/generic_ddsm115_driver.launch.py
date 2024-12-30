from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from os import path


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ros2_ddsm115_driver")
    rosparam_file = path.join(pkg_share, "config", "generic_ddsm115_driver_config.yaml")
    return LaunchDescription([
        Node(
            package="ros2_ddsm115_driver",
            executable="generic_ddsm115_driver",
            output="screen",
            parameters=[rosparam_file]
        )
    ])