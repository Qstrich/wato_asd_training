#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node
{
public:
  ControlNode();

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void control_loop();
  void publish_stop();
  bool frames_compatible() const;

  robot::ControlCore control_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Path::SharedPtr current_path_;
  nav_msgs::msg::Odometry::SharedPtr robot_odom_;
};

#endif
