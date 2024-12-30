from launch import LaunchDescription
from os.path import join
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ros2_rsbl_driver")
    ros_paramsfile = join(pkg_share, "config", "rsbl_calibrate.yaml")
    return LaunchDescription([
        Node(
            package="ros2_rsbl_driver",
            executable="calibrate_offset",
            output="screen",
            parameters=[ros_paramsfile]
        )
    ])