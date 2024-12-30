from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from os import path


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ros2_rsbl_driver")
    rosparam_file = path.join(pkg_share, "config", "rsbl_driver_config.yaml")
    return LaunchDescription([
        Node(
            package="ros2_rsbl_driver",
            executable="rsbl_driver",
            output="screen",
            parameters=[rosparam_file]
        )
    ])