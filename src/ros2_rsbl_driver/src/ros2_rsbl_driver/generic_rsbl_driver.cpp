#include <ros2_rsbl_driver/generic_rsbl_driver.hpp>

std::mutex ROS2_RSBL_DRIVER::m {};
ROS2_RSBL_DRIVER::ROS2_RSBL_DRIVER() : rclcpp::Node("rsbl_driver")
{
    RCLCPP_INFO(get_logger(), "ST-RSBL starting up");
    // Check if we have all required parameters
    declare_parameter<std::vector<std::string>>("motor_names");
    declare_parameter<std::vector<long int>>("motor_ids");
    declare_parameter<std::vector<std::string>>("motor_directions");
    declare_parameter<long int>("baud_rate");
    declare_parameter<std::string>("port", DEFAULT_SERIAL_PORT);
    if (!has_parameter("motor_names"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without motor_names parameter");
        rclcpp::shutdown();
    }
    else if (!has_parameter("motor_ids"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without motor_ids parameter");
        rclcpp::shutdown();
    }
    else if (!has_parameter("motor_directions"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without motor_directions parameter");
        rclcpp::shutdown();
    }
    else if (!has_parameter("baud_rate"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without baud_rate parameter");
        rclcpp::shutdown();
    }
    else if (!has_parameter("port"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without port parameter");
        rclcpp::shutdown();
    }
    else
    {
        // Get all parameters
        get_parameter<std::vector<std::string>>("motor_names", motor_names_list);
        get_parameter<std::vector<long int>>("motor_ids", motor_ids_list);
        get_parameter<std::vector<std::string>>("motor_directions", motor_directions_list);
        long int baud_rate;
        std::string port;
        get_parameter<long int>("baud_rate", baud_rate);
        get_parameter<std::string>("port", port);

        // Check that wheel array parameters have equial length (specify all wheels)
        if (!(motor_names_list.size() == motor_ids_list.size() && motor_ids_list.size() == motor_directions_list.size()))
        {
            RCLCPP_ERROR(get_logger(), "motor_names, motor_ids and motor_directions must be of an equal size");
            rclcpp::shutdown();
        }
        // Check that wheel names and ids are distinct (can't run with 2 wheels with the same name or id)
        else if (!stringsAreDistinct(motor_names_list))
        {
            RCLCPP_ERROR(get_logger(), "motor names must be distinct");
            rclcpp::shutdown();
        }
        else if (!intsAreDistinct(motor_ids_list))
        {
            RCLCPP_ERROR(get_logger(), "motor ids must be distinct");
            rclcpp::shutdown();
        }
        else if (!driver_.begin(baud_rate, port.c_str()))
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize RSBL-ST communication at port: %s", port.c_str());
            rclcpp::shutdown();
        }
        else
        {
            // Do all setup for every wheel
            for (size_t i = 0; i < motor_names_list.size(); i++)
            {
                // Check that parameters are valid
                if (motor_names_list[i].length() == 0)
                {
                    RCLCPP_ERROR(get_logger(), "motor_names can't be empty");
                    rclcpp::shutdown();
                }
                else if (motor_ids_list[i] <= 0)
                {
                    RCLCPP_ERROR(get_logger(), "motor_ids must be >0");
                    rclcpp::shutdown();
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                                "Adding wheel %s id %ld and direction %s",
                                motor_names_list[i].c_str(), motor_ids_list[i],
                                motor_directions_list[i].c_str());
                    // Create motor direction multiplyers array
                    if (motor_directions_list[i] == "clkwise")
                    {
                        motor_directions.push_back(-1);
                    }
                    else
                    {
                        motor_directions.push_back(1);
                    }
                    /*
                    Create motor target velocity subscriber, bind with motor name as additional parameter
                    to handle all motor with single callback
                    */
                    motor_pos_subs.push_back(
                        create_subscription<std_msgs::msg::Float64>(
                            motor_names_list[i] + "/target_pos", 1,
                            [this, motor_name = motor_names_list[i]](const std_msgs::msg::Float64::SharedPtr msg)
                            {
                                this->motorPosCallback(msg, motor_name);
                            }
                            ));

                    motor_pos_pubs.push_back(create_publisher<std_msgs::msg::Float64>(motor_names_list[i] + "/current_pos", 1));

                    motor_current_pubs.push_back(create_publisher<std_msgs::msg::Float64>(motor_names_list[i] + "/current", 1));

                }
            }
            auto timer_period = std::chrono::milliseconds(100); // 10 Hz
            timer = create_wall_timer(timer_period, [this](){this->publish_pos();});
        }
    }
}

ROS2_RSBL_DRIVER::~ROS2_RSBL_DRIVER()
{
    onShutdown();
}

void ROS2_RSBL_DRIVER::onShutdown()
{
    RCLCPP_INFO(get_logger(), "ST-RSBL shutting down");
    // stop all wheels
    for (const long int &motor_id : motor_ids_list)
    {
        driver_.WriteSpe(motor_id, 0, 100);
    }
    driver_.end();
}

bool ROS2_RSBL_DRIVER::stringsAreDistinct(const std::vector<std::string> &arr)
{
    std::unordered_set<std::string> s;
    for (const std::string &str : arr)
    {
        s.insert(str);
    }
    return s.size() == arr.size();
}

bool ROS2_RSBL_DRIVER::intsAreDistinct(const std::vector<long int> &arr)
{
    std::unordered_set<long int> s;
    for (const long int &elem : arr)
    {
        s.insert(elem);
    }
    return s.size() == arr.size();
}

long int ROS2_RSBL_DRIVER::motorIndexByName(const std::string &motor_name)
{
    std::vector<std::string>::iterator itr = std::find(motor_names_list.begin(), motor_names_list.end(), motor_name);
    if (itr != motor_names_list.end())
    {
        return std::distance(motor_names_list.begin(), itr);
    }
    else
    {
        return -1;
    }
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ROS2_RSBL_DRIVER::motorPosCallback(const std_msgs::msg::Float64::SharedPtr msg, const std::string &motor_name)
{
    // std_msgs::msg::Float64 velocity_msg;
    std_msgs::msg::Float64 pos_msg;
    std_msgs::msg::Float64 current_msg;
    int motor_index = motorIndexByName(motor_name);
    if (motor_index < 0)
    {
        RCLCPP_WARN(get_logger(), "Received target position for unknown motor %s", motor_name.c_str());
        return;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Received target position for motor: %s, pos: %lf", motor_name.c_str(), msg->data);
    }
    
    // for motors, cckwise is +ve but for nav stack clkwise is +ve
    int pos {static_cast<int>(map(msg->data, 45 * motor_directions[motor_index], 45 * -1 * motor_directions[motor_index], 2560, 1536))};
    m.lock();
    int response = driver_.WritePosEx(motor_ids_list[motor_index], pos, 2400, 100);
    m.unlock();
    // 100 is the acceleration in 100 steps / s^2, i.e. 10^4 steps/s^2
    // at 2400 steps/s , max speed
    if (!response)
    {
        RCLCPP_WARN(get_logger(), "failed to write position to motor - %s", motor_name.c_str());
    }
}

void ROS2_RSBL_DRIVER::publish_pos()
{
    std_msgs::msg::Float64 pos_msg;
    std_msgs::msg::Float64 current_msg;
    for(uint i{}; i < motor_names_list.size(); i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        m.lock();
        if (driver_.FeedBack(motor_ids_list[i]) != -1)
        {
            pos_msg.data = driver_.ReadPos(motor_ids_list[i]);
            pos_msg.data = map(pos_msg.data, 2560, 1536, 45 * motor_directions[i], 45 * -1 * motor_directions[i]);
            // velocity_msg.data = driver_.ReadSpeed(motor_ids_list[motor_index]) * motor_directions[motor_index];
            current_msg.data = driver_.ReadCurrent(motor_ids_list[i]);
            motor_pos_pubs[i]->publish(pos_msg);
            motor_current_pubs[i]->publish(current_msg);
            m.unlock();
            continue;
        }
        RCLCPP_WARN(get_logger(), "Failed to read from motor: %s with id: %ld", motor_names_list[i].c_str(), motor_ids_list[i]);
        m.unlock();
    }
}