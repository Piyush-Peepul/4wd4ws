#ifndef _ROS2_RSBL_DRIVER_HPP_
#define _ROS2_RSBL_DRIVER_HPP_
#include <SCServo_Linux/SMS_STS.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <unordered_set>
#include <chrono>
#include <mutex>

#define DEFAULT_SERIAL_PORT "/dev/serial/by-id/usb-1a86_USB_Single_Serial_58CD181205-if00"

class ROS2_RSBL_DRIVER : public rclcpp::Node
{
public:
    ROS2_RSBL_DRIVER();
    // TODO: implement
    ~ROS2_RSBL_DRIVER();

private:
    SMS_STS driver_;
    std::vector<long int> motor_ids_list;
    std::vector<std::string> motor_directions_list;
    std::vector<std::string> motor_names_list;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> motor_pos_subs;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> motor_pos_pubs;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> motor_current_pubs;
    std::vector<int> motor_directions;
    rclcpp::TimerBase::SharedPtr timer;
    static std::mutex m;

    void onShutdown();
    bool stringsAreDistinct(const std::vector<std::string> &arr);
    bool intsAreDistinct(const std::vector<long int> &arr);
    long int motorIndexByName(const std::string &motor_name);
    void motorPosCallback(const std_msgs::msg::Float64::SharedPtr msg, const std::string &motor_name);
    void publish_pos();
};

#endif