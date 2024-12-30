#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>

// TODO: complete

class TestServos : public rclcpp::Node
{
public:
    TestServos() : rclcpp::Node("test_servos")
    {
        declare_parameter<std::string>("port");
        declare_parameter<long int>("baud_rate");
        declare_parameter<std::vector<long int>>("motor_ids");
        if(!has_parameter("port"))
        {
            RCLCPP_ERROR(get_logger(), "Can't run without port parameter!");
        }
        else if(!has_parameter("baud_rate"))
        {
            RCLCPP_ERROR(get_logger(), "Can't run without baud_rate parameter");
        }
        else if(!has_parameter("motor_ids"))
        {
            RCLCPP_ERROR(get_logger(), "Can't run without motor_ids parameter");
        }
        else if(!driver.begin(baud_rate, port.c_str()))
        {
            RCLCPP_ERROR(get_logger(), "Failed to open port: %s with baud_rate: %ld", port.c_str(), baud_rate);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Opened port: %s with baud_rate: %ld", port.c_str(), baud_rate);
        }
    }
private:
    rclcpp::TimerBase::SharedPtr timer;
    SMS_STS driver;

    void pub_vel()
    {
        rclcpp::Rate r{50};
        for(uint j{}; j < pubs.size(); j++)
        {
            pubs[j]->publish(positions[ind]);
            r.sleep();
        }
        ind++;
        if(static_cast<uint>(ind) >= positions.size()) ind = 0;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestServos>());
    rclcpp::shutdown();
    return 0;
}