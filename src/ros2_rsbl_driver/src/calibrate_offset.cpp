#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <ros2_rsbl_driver/generic_rsbl_driver.hpp>

class CalibrateServoOffsets: public rclcpp::Node
{
public:
    CalibrateServoOffsets() : rclcpp::Node("calibrate_servo_offsets")
    {
        declare_parameter<std::string>("port", "ttyACM0");
        declare_parameter<long int>("baud_rate", 1000000);
        if(!has_parameter("port"))
        {
            RCLCPP_ERROR(get_logger(), "Can't run without port parameter");
        }
        else if(!has_parameter("baud_rate"))
        {
            RCLCPP_ERROR(get_logger(), "Can't run without baud_rate parameter");
        }
        else
        {
            std::string port;
            long int baud_rate;
            get_parameter<std::string>("port", port);
            get_parameter<long int>("baud_rate", baud_rate);
            if(!driver.begin(baud_rate, port.c_str()))
            {
                RCLCPP_ERROR(get_logger(), "Couldn't open the serial port %s @ %ld buard rate", port.c_str(), baud_rate);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Will now ping the motors sequentially from ID = 1 to ID = 253");
                rclcpp::Rate r{25}; // 40 ms delay
                for(int i{1}; i < 254 && rclcpp::ok(); i++)
                {
                    RCLCPP_INFO(get_logger(), "Pinging ID: %d", i);
                    int ID = driver.Ping(i);
                    if(ID > 0)
                    {
                        RCLCPP_INFO(get_logger(), "Got ID: %d. Calibrating the offset now", ID);
                        driver.CalibrationOfs(i);
                        if(-1 != driver.FeedBack(i))
                        {
                            int pos = driver.ReadPos(i);
                            if(2048 != pos)
                            {
                                RCLCPP_ERROR(get_logger(), "Callibration for motor ID: %d failed!, pos: %d", i, pos);
                            }
                            else
                            {
                                RCLCPP_INFO(get_logger(), "Calibration of motor ID %d successful!", i);
                            }
                        }
                        else
                            RCLCPP_ERROR(get_logger(), "Couldn't read back from motor ID: %d!", i);
                    }
                    r.sleep();
                }
            }
        }
    }
private:
    SMS_STS driver;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node {std::make_shared<CalibrateServoOffsets>()};
    rclcpp::shutdown();
}