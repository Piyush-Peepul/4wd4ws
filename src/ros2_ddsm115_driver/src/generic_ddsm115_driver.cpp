#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <unordered_set>
#include <math.h>
#include <string>
#include <mutex>
#include <ros2_ddsm115_driver/ddsm115_communicator.hpp>

#define DEFAULT_SERIAL_PORT "/dev/ttyACM0"
#define NODE_NAME "generic_ddsm115_driver"

class ROS2_DDSM_DRIVER : public rclcpp::Node
{
public:
    ROS2_DDSM_DRIVER();
    ~ROS2_DDSM_DRIVER();

private:
    DDSM115Communicator *driver;
    std::vector<long int> wheel_ids_list;
    std::vector<std::string> wheel_directions_list;
    std::vector<std::string> wheel_names_list;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> wheel_velocity_subs;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> wheel_velocity_pubs;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> wheel_angle_pubs;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> wheel_current_pubs;
    std::vector<int> wheel_directions;
    rclcpp::TimerBase::SharedPtr timer;

    std::vector<ddsm115_drive_response> responses;

    // callbacks
    void wheelVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg, const std::string &wheel_name);
    void onShutdown();
    void pub_vel();

    bool stringsAreDistinct(const std::vector<std::string> &arr);
    bool intsAreDistinct(const std::vector<long int> &arr);
    int wheelIndexByName(const std::string &wheel_name);
    double linVel2Rpm(double velocity);
    double rpm2LinVel(double rpm);

    const double WHEEL_RADIUS {0.051/2}; // meter

    std::mutex m;
};

double ROS2_DDSM_DRIVER::linVel2Rpm(double velocity)
{
    return (velocity / WHEEL_RADIUS) / (2.0 * M_PI) * 60.0;
}

double ROS2_DDSM_DRIVER::rpm2LinVel(double rpm)
{
    return (rpm * 2.0 * M_PI / 60.0) * WHEEL_RADIUS;
}

int ROS2_DDSM_DRIVER::wheelIndexByName(const std::string &wheel_name)
{
    std::vector<std::string>::iterator itr = std::find(wheel_names_list.begin(), wheel_names_list.end(), wheel_name);
    if (itr != wheel_names_list.end())
    {
        return std::distance(wheel_names_list.begin(), itr);
    }
    else
    {
        return -1;
    }
}

void ROS2_DDSM_DRIVER::wheelVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg, const std::string &wheel_name)
{
    int wheel_index = wheelIndexByName(wheel_name);
    if (wheel_index < 0)
    {
        RCLCPP_WARN(get_logger(), "Received target velocity for unknown wheel %s", wheel_name.c_str());
        return;
    }
    m.lock();
    responses[wheel_index] = driver->setWheelRPM(
        wheel_ids_list[wheel_index], linVel2Rpm(msg->data * wheel_directions[wheel_index]));
    m.unlock();
    if(responses[wheel_index].result != DDSM115State::STATE_NORMAL)
    {
        RCLCPP_ERROR(get_logger(), "Failed to write velocity to wheel: %s, ID: %ld", wheel_name.c_str(), wheel_ids_list[wheel_index]);
    }
    // DEBUG:
    // else
    // {
    //     RCLCPP_INFO(get_logger(), "Received target velocity for wheel: %s, velocity: %lf", wheel_name.c_str(), msg->data);
    // }
}

void ROS2_DDSM_DRIVER::pub_vel()
{
    // TODO: add timestamp
    std_msgs::msg::Float64 velocity_msg;
    std_msgs::msg::Float64 angle_msg;
    std_msgs::msg::Float64 current_msg;
    m.lock();
    std::vector<ddsm115_drive_response> responses {this->responses};
    m.unlock();
    for(uint i{}; i < responses.size(); i++)
    {
        auto &res{responses[i]};
        if (res.result == DDSM115State::STATE_NORMAL)
        {
            // DEBUG:
            // RCLCPP_INFO(get_logger(), "raw vel feedback: %lf", res.velocity);
            velocity_msg.data = rpm2LinVel(res.velocity) * wheel_directions[i];
            // TODO: corss check formula for angle.
            angle_msg.data = round(res.position * (2.0 * M_PI / 360.0) * wheel_directions[i] * 100) / 100 * -1.0;
            current_msg.data = res.current;
            wheel_velocity_pubs[i]->publish(velocity_msg);
            wheel_angle_pubs[i]->publish(angle_msg);
            wheel_current_pubs[i]->publish(current_msg);
            res.result = DDSM115State::STATE_FAILED; // avoid repeated publishing
        }
    }
}

bool ROS2_DDSM_DRIVER::stringsAreDistinct(const std::vector<std::string> &arr)
{
    std::unordered_set<std::string> s;
    for (const std::string &str : arr)
    {
        s.insert(str);
    }
    return s.size() == arr.size();
}

bool ROS2_DDSM_DRIVER::intsAreDistinct(const std::vector<long int> &arr)
{
    std::unordered_set<int> s;
    for (const long int &elem : arr)
    {
        s.insert(elem);
    }
    return s.size() == arr.size();
}

void ROS2_DDSM_DRIVER::onShutdown()
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

ROS2_DDSM_DRIVER::~ROS2_DDSM_DRIVER()
{
    onShutdown();
    delete driver;
}

ROS2_DDSM_DRIVER::ROS2_DDSM_DRIVER()
    : rclcpp::Node(NODE_NAME)
{
    RCLCPP_INFO(get_logger(), "DDSM115 starting up");
    // Check if we have all required parameters

    // ALWAYS DECLARE PARAMETERS BEFORE CALLING rclcpp::Node::has_parameter() otherwise it will
    // always return false!
    this->declare_parameter<std::vector<std::string>>("wheel_names");
    this->declare_parameter<std::vector<long int>>("wheel_ids");
    this->declare_parameter<std::vector<std::string>>("wheel_directions");
    this->declare_parameter<std::string>("port_name", DEFAULT_SERIAL_PORT);
    if (!this->has_parameter("wheel_names"))
    {
        RCLCPP_ERROR(get_logger(), "Can't run without wheel_names parameter");
        rclcpp::shutdown();
    }
    else if (!this->has_parameter("wheel_ids"))
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
        this->get_parameter<std::vector<std::string>>("wheel_names", wheel_names_list);
        this->get_parameter<std::vector<long int>>("wheel_ids", wheel_ids_list);
        this->get_parameter<std::vector<std::string>>("wheel_directions", wheel_directions_list);

        std::string port_name;
        this->get_parameter<std::string>("port_name", port_name);
        driver = new DDSM115Communicator(port_name);

        // Check that wheel array parameters have equial length (specify all wheels)
        if (!(wheel_names_list.size() == wheel_ids_list.size() && wheel_ids_list.size() == wheel_directions_list.size()))
        {
            RCLCPP_ERROR(get_logger(), "wheel_names, wheel_ids and wheel_directions must be of an equal size");
            rclcpp::shutdown();
        }
        // Check that wheel names and ids are distinct (can't run with 2 wheels with the same name or id)
        else if (!stringsAreDistinct(wheel_names_list))
        {
            RCLCPP_ERROR(get_logger(), "wheel names must be distinct");
            rclcpp::shutdown();
        }
        else if (!intsAreDistinct(wheel_ids_list))
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
        else
        {
            // Do all setup for every wheel
            for (int i = 0; i < (int)wheel_names_list.size(); i++)
            {
                // Check that parameters are valid
                if (wheel_names_list[i].length() == 0)
                {
                    RCLCPP_ERROR(get_logger(), "wheel name can't be empty");
                    rclcpp::shutdown();
                }
                else if (wheel_ids_list[i] <= 0)
                {
                    RCLCPP_ERROR(get_logger(), "wheel id must be >0");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(get_logger(),
                            "Adding wheel %s id %ld and direction %s",
                            wheel_names_list[i].c_str(), wheel_ids_list[i],
                            wheel_directions_list[i].c_str());
                // Create wheel direction multiplyers array
                if (wheel_directions_list[i] == "forward")
                {
                    wheel_directions.push_back(1);
                }
                else
                {
                    wheel_directions.push_back(-1);
                }
                /*
                Create wheel target velocity subscriber, bind with wheel name as additional parameter
                to handle all wheels with single callback
                */
                wheel_velocity_subs.push_back(
                    create_subscription<std_msgs::msg::Float64>(
                        wheel_names_list[i] + "/target_velocity", 1,
                        [this, wheel_name = wheel_names_list[i]](const std_msgs::msg::Float64::SharedPtr msg)
                        {
                            this->wheelVelocityCallback(msg, wheel_name);
                        }
                        ));

                wheel_velocity_pubs.push_back(create_publisher<std_msgs::msg::Float64>(wheel_names_list[i] + "/current_velocity", 1));

                wheel_angle_pubs.push_back(create_publisher<std_msgs::msg::Float64>(wheel_names_list[i] + "/angle", 1));

                wheel_current_pubs.push_back(create_publisher<std_msgs::msg::Float64>(wheel_names_list[i] + "/current", 1));

                // Set wheel mode to velocity loop
                driver->setWheelMode(wheel_ids_list[i], DDSM115Mode::VELOCITY_LOOP);

            }
            timer = create_wall_timer(std::chrono::milliseconds(100), [this](){this->pub_vel();});
            responses.resize(wheel_names_list.size());
        }
    }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node {std::make_shared<ROS2_DDSM_DRIVER>()};
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}