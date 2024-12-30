from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from os.path import join


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("ros2_rsbl_driver")
    rosparms_file = join(pkg_share, "config", "test_servos.yaml")

    return LaunchDescription([
        Node(
            package="ros2_rsbl_driver",
            executable="test_servos",
            output="screen",
            parameters=[rosparms_file]
        )
    ])