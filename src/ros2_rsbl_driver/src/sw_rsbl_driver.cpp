#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ros2_rsbl_driver/generic_rsbl_driver.hpp>
#include <unordered_set>

#define SERIAL_PORT "/dev/ttyUSB1"

class RSBL_DRIVER : public rclcpp::Node
{
public:
    RSBL_DRIVER();
    ~RSBL_DRIVER();
private:
    SMS_STS driver;
    std::vector<long int> motor_ids_list;
    std::vector<std::string> motor_directions_list;
    std::vector<int> motor_directions;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pos_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pos_pub;
    rclcpp::TimerBase::SharedPtr timer;

    void motorPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void onShutdown();
    void pub_pos();

    bool intsAreDistinct(const std::vector<long int> &arr);
    double map(double x, double in_min, double in_max, double out_min, double out_max);

    std::mutex m;
};

RSBL_DRIVER::RSBL_DRIVER()
    : rclcpp::Node("sw_rsbl_driver")
{
    RCLCPP_INFO(get_logger(), "SW RSBL starting up");

    this->declare_parameter<std::vector<long int>>("motor_ids");
    this->declare_parameter<std::vector<std::string>>("motor_directions");
    this->declare_parameter<std::string>("port", SERIAL_PORT);
    this->declare_parameter<long int>("baud_rate", 1000000);
    if (!this->has_parameter("motor_ids"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without motor_ids parameter");
        rclcpp::shutdown();
    }
    else if (!this->has_parameter("motor_directions"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without motor_directions parameter");
        rclcpp::shutdown();
    }
    else if (!this->has_parameter("port"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without port parameter");
        rclcpp::shutdown();
    }
    else
    {
        this->get_parameter<std::vector<long int>>("motor_ids", motor_ids_list);
        this->get_parameter<std::vector<std::string>>("motor_directions", motor_directions_list);

        std::string port;
        long int baud_rate;
        this->get_parameter<long int>("baud_rate", baud_rate);
        this->get_parameter<std::string>("port", port);

        if(!driver.begin(baud_rate, port.c_str()))
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize RSBL communication");
            rclcpp::shutdown();
        }
        else if (!intsAreDistinct(motor_ids_list))
        {
            RCLCPP_ERROR(get_logger(), "motor ids must be distinct");
            rclcpp::shutdown();
        }
        else if (motor_ids_list.size() != 4)
        {
            RCLCPP_ERROR(get_logger(), "motor_ids must have 4 elements");
            rclcpp::shutdown();
        }
        else if(motor_directions_list.size() != 4)
        {
            RCLCPP_ERROR(get_logger(), "motor_directions must have 4 elements");
            rclcpp::shutdown();
        }
        else
        {
            for (uint i = 0; i < 4u; i++)
            {
                if (motor_ids_list[i] <= 0)
                {
                    RCLCPP_ERROR(get_logger(), "motor id must > 0");
                }
                if(motor_directions_list[i] == "clkwise")
                {
                    motor_directions.push_back(-1);
                }
                else
                {
                    motor_directions.push_back(1);
                }
            }
        }
        motor_pos_sub = create_subscription<std_msgs::msg::Float64MultiArray>(
            "cmd_wheel_steering", 1,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
            {
                this->motorPosCallback(msg);
            }
        );
        motor_pos_pub = create_publisher<std_msgs::msg::Float64MultiArray>("current_wheel_steering", 1);
        timer = create_wall_timer(std::chrono::milliseconds(100), [this](){this->pub_pos();});
        // DEBUG
        RCLCPP_INFO(get_logger(), "RSBL initialized");
    }
}

RSBL_DRIVER::~RSBL_DRIVER()
{
    onShutdown();
}

void RSBL_DRIVER::onShutdown()
{
    RCLCPP_INFO(get_logger(), "RSBL shutting down");
    for (const long int &motor_id : motor_ids_list)
    {
        driver.WriteSpe(motor_id, 0, 100);
    }
    driver.end();
}


double RSBL_DRIVER::map(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void RSBL_DRIVER::motorPosCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 4) {
        RCLCPP_ERROR(get_logger(), "Received message with insufficient data");
        return;
    }

    std::lock_guard<std::mutex> lock(m);
    for (uint i = 0; i < 4u; i++)
    {
        // for motors, cckwise is +ve but for nav stack clkwise is +ve
        int pos {static_cast<int>(map(msg->data[i], 45 * motor_directions[i], 45 * -1 * motor_directions[i], 2560, 1536))};
        driver.WritePosEx(motor_ids_list[i], pos, 2400, 100); // returns 0 on failure
    }
}

void RSBL_DRIVER::pub_pos()
{
    std_msgs::msg::Float64MultiArray pos_msg;
    std_msgs::msg::Float64MultiArray current_msg;
    for (uint i = 0; i < 4u; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::lock_guard<std::mutex> lock(m);
        if (driver.FeedBack(motor_ids_list[i]) != -1)
        {
            pos_msg.data.push_back(map(driver.ReadPos(motor_ids_list[i]), 2560, 1536, 45 * motor_directions[i], 45 * -1 * motor_directions[i]));
            current_msg.data.push_back(driver.ReadCurrent(motor_ids_list[i]));
        }
    }
    motor_pos_pub->publish(pos_msg);
}

bool RSBL_DRIVER::intsAreDistinct(const std::vector<long int> &arr)
{
    std::unordered_set<long int> s;
    for (const long int &elem : arr)
    {
        s.insert(elem);
    }
    return s.size() == arr.size();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RSBL_DRIVER>());
    rclcpp::shutdown();
    return 0;
}