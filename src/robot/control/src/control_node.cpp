#include <algorithm>
#include <chrono>
#include <functional>

#include "control_node.hpp"

ControlNode::ControlNode()
: Node("control_node"),
  control_(
    this->get_logger(),
    this->declare_parameter<double>("lookahead_distance", 1.0),
    this->declare_parameter<double>("goal_tolerance", 0.1),
    this->declare_parameter<double>("linear_speed", 0.5),
    this->declare_parameter<double>("max_angular_speed", 1.5))
{
  const int timer_period_ms = std::max(
    1,
    static_cast<int>(this->declare_parameter<int>("timer_period_ms", 100)));

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path",
    10,
    std::bind(
      &ControlNode::path_callback,
      this,
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(
      &ControlNode::odom_callback,
      this,
      std::placeholders::_1));
  cmd_vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_period_ms),
    std::bind(&ControlNode::control_loop, this));
}

void ControlNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = msg;
}

void ControlNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_odom_ = msg;
}

void ControlNode::control_loop()
{
  if (!current_path_ || !robot_odom_ || !frames_compatible()) {
    publish_stop();
    return;
  }

  const robot::ControlResult result =
    control_.compute_command(*current_path_, *robot_odom_);
  if (result.stop) {
    publish_stop();
    return;
  }
  cmd_vel_pub_->publish(result.command);
}

void ControlNode::publish_stop()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
}

bool ControlNode::frames_compatible() const
{
  if (!current_path_ || !robot_odom_) {
    return false;
  }
  return current_path_->header.frame_id.empty() ||
    robot_odom_->header.frame_id.empty() ||
    current_path_->header.frame_id == robot_odom_->header.frame_id;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
