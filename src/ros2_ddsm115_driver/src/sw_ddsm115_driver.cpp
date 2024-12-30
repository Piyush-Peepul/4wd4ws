#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <unordered_set>
#include <math.h>
#include <string>
#include <mutex>
#include <ros2_ddsm115_driver/ddsm115_communicator.hpp>
// #include <ros2_ddsm115_driver/msg/float64_array.hpp>

#define DEFAULT_SERIAL_PORT "/dev/ttyACM0"
#define NODE_NAME "sw_ddsm115_driver"

class SW_DDSM_DRIVER : public rclcpp::Node
{
public:
    SW_DDSM_DRIVER();
    ~SW_DDSM_DRIVER();

private:
    DDSM115Communicator *driver;
    std::vector<long int> wheel_ids_list;
    std::vector<std::string> wheel_directions_list;
    std::vector<int> wheel_directions;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_velocity_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_velocity_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_current_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pos_pub;
    rclcpp::TimerBase::SharedPtr timer;

    std::vector<ddsm115_drive_response> responses;

    // callbacks
    // void wheelVelocityCallback(const ros2_ddsm115_driver::msg::Float64Array::SharedPtr msg);
    void wheelVelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void onShutdown();
    void pub_vel();

    bool intsAreDistinct(const std::vector<long int> &arr);
    double linVel2Rpm(double velocity);
    double rpm2LinVel(double rpm);

    const double WHEEL_RADIUS {0.051/2}; // meter

    std::mutex m;
};

double SW_DDSM_DRIVER::linVel2Rpm(double velocity)
{
    return (velocity / WHEEL_RADIUS) / (2.0 * M_PI) * 60.0;
}

double SW_DDSM_DRIVER::rpm2LinVel(double rpm)
{
    return (rpm * 2.0 * M_PI / 60.0) * WHEEL_RADIUS;
}

// void SW_DDSM_DRIVER::wheelVelocityCallback(const ros2_ddsm115_driver::msg::Float64Array::SharedPtr msg)
void SW_DDSM_DRIVER::wheelVelocityCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if(msg->data.size() > 4)
    {
        RCLCPP_WARN(get_logger(), "Received more than 4 wheel velocities, ignoring extra values");
    }
    else if(msg->data.size() < 4)
    {
        RCLCPP_ERROR(get_logger(), "Received less than 4 wheel velocities, ignoring entire message");
        return;
    }
    m.lock();
    for (int i{}; i < 4; i++)
    {
        responses[i] = driver->setWheelRPM(
            wheel_ids_list[i], linVel2Rpm(msg->data[i] * wheel_directions[i]));
        if(responses[i].result != DDSM115State::STATE_NORMAL)
        {
            RCLCPP_ERROR(get_logger(), "Failed to write velocity to wheel ID: %ld", wheel_ids_list[i]);
        }
    }
    m.unlock();
}

void SW_DDSM_DRIVER::pub_vel()
{
    // TODO: add timestamp
    std_msgs::msg::Float64MultiArray velocity_msg;
    std_msgs::msg::Float64MultiArray current_msg;
    std_msgs::msg::Float64MultiArray pos_msg;
    m.lock();
    std::vector<ddsm115_drive_response> responses {this->responses};
    m.unlock();
    for(uint i{}; i < 4; i++)
    {
        auto &res{responses[i]};
        if (res.result == DDSM115State::STATE_NORMAL)
        {
            // DEBUG:
            // RCLCPP_INFO(get_logger(), "raw vel feedback: %lf", res.velocity);
            velocity_msg.data.push_back(rpm2LinVel(res.velocity) * wheel_directions[i]);
            // TODO: corss check formula for angle.
            pos_msg.data.push_back((res.position * (2.0 * M_PI / 360.0) * wheel_directions[i] * 100) / 100 * -1.0);
            current_msg.data.push_back(res.current);
            res.result = DDSM115State::STATE_FAILED; // avoid repeated computation
        }
    }
    wheel_velocity_pub->publish(velocity_msg);
    // wheel_pos_pub->publish(pos_msg);
    wheel_current_pub->publish(current_msg);
}

bool SW_DDSM_DRIVER::intsAreDistinct(const std::vector<long int> &arr)
{
    std::unordered_set<int> s;
    for (const long int &elem : arr)
    {
        s.insert(elem);
    }
    return s.size() == arr.size();
}

void SW_DDSM_DRIVER::onShutdown()
{
    RCLCPP_INFO(get_logger(), "DDSM115 shutting down");
    // stop all wheels
    rclcpp::Rate r{5};
    for (const long int &wheel_id : wheel_ids_list)
    {
        driver->setWheelRPM(wheel_id, 0.0);
        r.sleep();
    }
    driver->disconnect();
}

SW_DDSM_DRIVER::~SW_DDSM_DRIVER()
{
    onShutdown();
    delete driver;
}

SW_DDSM_DRIVER::SW_DDSM_DRIVER()
    : rclcpp::Node(NODE_NAME)
{
    RCLCPP_INFO(get_logger(), "DDSM115 starting up");
    // Check if we have all required parameters

    // ALWAYS DECLARE PARAMETERS BEFORE CALLING rclcpp::Node::has_parameter() otherwise it will
    // always return false!
    this->declare_parameter<std::vector<long int>>("wheel_ids");
    this->declare_parameter<std::vector<std::string>>("wheel_directions");
    this->declare_parameter<std::string>("port_name", DEFAULT_SERIAL_PORT);
    if (!this->has_parameter("wheel_ids"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without wheel_ids parameter");
        rclcpp::shutdown();
    }
    else if (!this->has_parameter("wheel_directions"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without wheel_directions parameter");
        rclcpp::shutdown();
    }
    else if (!this->has_parameter("port_name"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without port_name parameter");
        rclcpp::shutdown();
    }
    else
    {
        // Get all parameters
        this->get_parameter<std::vector<long int>>("wheel_ids", wheel_ids_list);
        this->get_parameter<std::vector<std::string>>("wheel_directions", wheel_directions_list);

        std::string port_name;
        this->get_parameter<std::string>("port_name", port_name);
        driver = new DDSM115Communicator(port_name);

        if (!intsAreDistinct(wheel_ids_list))
        {
            RCLCPP_ERROR(get_logger(), "wheel ids must be distinct");
            rclcpp::shutdown();
        }
        // Open DDSM115 communication interface
        else if (driver->getState() != DDSM115State::STATE_NORMAL)
        {
            RCLCPP_ERROR(get_logger(), "Failed to initialize DDSM115 communication");
            rclcpp::shutdown();
        }
        // Check that parameters are valid
        if (wheel_ids_list.size() != 4)
        {
            RCLCPP_ERROR(get_logger(), "wheel_ids must have 4 elements");
            rclcpp::shutdown();
        }
        else if (wheel_ids_list.size() != 4)
        {
            RCLCPP_ERROR(get_logger(), "wheel_ids must have 4 elements");
            rclcpp::shutdown();
        }
        else
        {
            // Do all setup for every wheel
            for (uint i = 0; i < 4u; i++)
            {
                if (wheel_ids_list[i] <= 0)
                {
                    RCLCPP_ERROR(get_logger(), "wheel id must be > 0");
                    rclcpp::shutdown();
                }
                // Create wheel direction multiplyers array
                if (wheel_directions_list[i] == "forward")
                {
                    wheel_directions.push_back(1);
                }
                else
                {
                    wheel_directions.push_back(-1);
                }

                // Set wheel mode to velocity loop
                driver->setWheelMode(wheel_ids_list[i], DDSM115Mode::VELOCITY_LOOP);
            }
            // setup publishers and subscribers
            wheel_velocity_sub = create_subscription<std_msgs::msg::Float64MultiArray>("cmd_wheel_vel", 1, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg){this->wheelVelocityCallback(msg);});
            wheel_velocity_pub = create_publisher<std_msgs::msg::Float64MultiArray>("current_wheel_vel", 1);
            wheel_current_pub = create_publisher<std_msgs::msg::Float64MultiArray>("wheel_currents", 1);
            wheel_pos_pub = create_publisher<std_msgs::msg::Float64MultiArray>("wheel_positions", 1);
            timer = create_wall_timer(std::chrono::milliseconds(100), [this](){this->pub_vel();});
            responses.resize(4);
            // DEBUG
            RCLCPP_INFO(get_logger(), "DDSM115 initialized");
        }
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SW_DDSM_DRIVER>());
    rclcpp::shutdown();
    return 0;
}