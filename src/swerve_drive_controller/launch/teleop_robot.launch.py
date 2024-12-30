from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os import path

def generate_launch_description() -> LaunchDescription:
    ros2_ddsm115_driver_pkg_share = get_package_share_directory('ros2_ddsm115_driver')
    ros2_rsbl_driver_pkg_share = get_package_share_directory('ros2_rsbl_driver')
    ddsm115_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(ros2_ddsm115_driver_pkg_share, 'launch', 'sw_ddsm115_driver.launch.py')
        )
    )
    rsbl_driver_luaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(ros2_rsbl_driver_pkg_share, 'launch', 'sw_rsbl_driver.launch.py')
        )
    )
    return LaunchDescription([
        ddsm115_driver_launch,
        rsbl_driver_luaunch,
        Node(
            package='swerve_drive_controller',
            executable='sw_drive_controller',
            name='sw_drive_controller',
            output='screen',
            parameters=[path.join(ros2_rsbl_driver_pkg_share, 'config', 'sw_controller_config.yaml')]
        )
    ])