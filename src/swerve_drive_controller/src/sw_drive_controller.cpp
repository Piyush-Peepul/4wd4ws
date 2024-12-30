#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <atomic>
#include <mutex>

class SwerveDriveController : public rclcpp::Node
{
public:
  SwerveDriveController()
  : Node("swerve_drive_controller")
  {
    declare_parameter<double>("length", 0.5);
    declare_parameter<double>("width", 0.5);
    if(!this->has_parameter("length"))
    {
      RCLCPP_ERROR(this->get_logger(), "length parameter is required");
      rclcpp::shutdown();
    }
    else if(!this->has_parameter("width"))
    {
      RCLCPP_ERROR(this->get_logger(), "width parameter is required");
      rclcpp::shutdown();
    }
    else
    {
      double L = this->get_parameter("length").as_double();
      double W = this->get_parameter("width").as_double();
      double R = sqrt(L * L + W * W);
      W_BY_R_ = W / R;
      L_BY_R_ = L / R;
      vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg){
          this->velocity_callback(msg);
        });
      
      pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/current_wheel_steering", 1, [this](std_msgs::msg::Float64MultiArray::SharedPtr msg){
          this->pos_callback(msg);
        });

      cmd_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/cmd_wheel_vel", 1);
      cmd_steering_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/cmd_wheel_steering", 1);
      
      // ALWAYS INITIALIZE THE VECTORS
      zero_velocities_.data = {0.0, 0.0, 0.0, 0.0};
      cmd_velocities_.data = {0.0, 0.0, 0.0, 0.0};
      primary_angles_.data = {0.0, 0.0, 0.0, 0.0};
      // Watchdog timer, timeout is set to 1 second
      wathcdog_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this](){this->watchdog_callback();});

      RCLCPP_INFO(this->get_logger(), "Swerve Drive Controller has been started");
      RCLCPP_WARN(this->get_logger(), "Watchdog is active, please send velocity commands to keep the robot moving");
      RCLCPP_WARN(this->get_logger(), "If /current_steering is not published, the robot will not move");
    }
  }

private:
  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // kicking the watchdog
    watchdog_flag1_ = false;

    // meter
    double A = - msg->linear.y - msg->angular.z * W_BY_R_;
    double B = - msg->linear.y + msg->angular.z * W_BY_R_;
    double C = msg->linear.x - msg->angular.z * L_BY_R_;
    double D = msg->linear.x + msg->angular.z * L_BY_R_;

    // Wheel velocities
    double front_left_velocity = sqrt(B * B + D * D);
    double front_right_velocity = sqrt(B * B + C * C);
    double rear_left_velocity = sqrt(A * A + D * D);
    double rear_right_velocity = sqrt(A * A + C * C);

    // Wheel steering angles
    double front_left_angle = atan2(B, D) * 180 / M_PI;
    double front_right_angle = atan2(B, C) * 180 / M_PI;
    double rear_left_angle = atan2(A, D) * 180 / M_PI;
    double rear_right_angle = atan2(A, C) * 180 / M_PI;

    // Fill the message arrays
    // only primary values!
    m_.lock();
    cmd_velocities_.data = {front_left_velocity, front_right_velocity, rear_left_velocity, rear_right_velocity};
    primary_angles_.data = {front_left_angle, front_right_angle, rear_left_angle, rear_right_angle};
    m_.unlock();
  }

  void pos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr current_angles)
  {
    // kikcing the watchdog
    watchdog_flag2_ = false;
    // if we are not receiving velocity commands, do not send steering commands
    if(watchdog_flag1_)
    {
      return;
    }
    std_msgs::msg::Float64MultiArray cmd_angles;
    std_msgs::msg::Float64MultiArray primary_angles_;
    std_msgs::msg::Float64MultiArray cmd_velocities_;
    m_.lock();
    primary_angles_.data = this->primary_angles_.data;
    cmd_velocities_.data = this->cmd_velocities_.data;
    m_.unlock();
    for(int i{}; i < 4; i++)
    {
      // difference between where we are and where we want to be
      double diff {primary_angles_.data[i] - current_angles->data[i]};
      // normalize the difference
      if(diff < -90.0)
      {
        diff += 180.0;
        cmd_velocities_.data[i] = -cmd_velocities_.data[i];
      }
      else if(diff > 90.0)
      {
        diff -= 180.0;
        cmd_velocities_.data[i] = -cmd_velocities_.data[i];
      }
      // diff + current angle = new steering command
      cmd_angles.data.push_back(diff + current_angles->data[i]);
      // avoid twisting of hub motor cables
      cmd_angles.data[i] = (cmd_angles.data[i] > 180.0 ? cmd_velocities_.data[i] *= -1, cmd_angles.data[i] - 180.0 : (cmd_angles.data[i] < -180.0 ? cmd_velocities_.data[i] *= -1,  cmd_angles.data[i] + 180.0 : cmd_angles.data[i]));
    }
    cmd_steering_publisher_->publish(cmd_angles);
    m2_.lock();
    cmd_vel_publisher_->publish(cmd_velocities_);
    m2_.unlock();
  }

  void watchdog_callback()
  {
    if(watchdog_flag1_)
    {
      RCLCPP_WARN(this->get_logger(), "Watchdog bit, not receiving velocity commands");
      m2_.lock();
      cmd_vel_publisher_->publish(zero_velocities_);
      m2_.unlock();
    }
    watchdog_flag1_ = true;
    if(watchdog_flag2_)
    {
      RCLCPP_WARN(this->get_logger(), "Watchdog bit, not receiving angles");
      m2_.lock();
      cmd_vel_publisher_->publish(zero_velocities_);
      m2_.unlock();
    }
    watchdog_flag2_ = true;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_steering_publisher_;
  std_msgs::msg::Float64MultiArray primary_angles_;
  std_msgs::msg::Float64MultiArray cmd_velocities_;
  std::mutex m_;
  std::mutex m2_;
  rclcpp::TimerBase::SharedPtr wathcdog_timer_;
  std::atomic<bool> watchdog_flag1_{true};
  std::atomic<bool> watchdog_flag2_{true};
  std_msgs::msg::Float64MultiArray zero_velocities_;
  double W_BY_R_;
  double L_BY_R_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwerveDriveController>());
  rclcpp::shutdown();
  return 0;
}