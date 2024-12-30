#include <rclcpp/rclcpp.hpp>
#include <ros2_rsbl_driver/generic_rsbl_driver.hpp>

// TODO: replace with by-id
#define SERIAL_PORT "/dev/ttyUSB1"

// TODO: This is a "generic" driver exposing target_pos topics for individual servo. Write another one to interface with nav2

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROS2_RSBL_DRIVER>());
    rclcpp::shutdown();
    return 0;
}