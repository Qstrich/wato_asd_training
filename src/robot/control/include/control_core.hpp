#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

struct ControlResult
{
  geometry_msgs::msg::Twist command;
  bool stop;
};

class ControlCore
{
public:
  ControlCore(
    const rclcpp::Logger & logger,
    double lookahead_distance,
    double goal_tolerance,
    double linear_speed,
    double max_angular_speed);

  ControlResult compute_command(
    const nav_msgs::msg::Path & path,
    const nav_msgs::msg::Odometry & odometry) const;

private:
  static double distance(
    const geometry_msgs::msg::Point & first,
    const geometry_msgs::msg::Point & second);
  static double yaw_from_quaternion(
    const geometry_msgs::msg::Quaternion & quaternion);

  rclcpp::Logger logger_;
  double lookahead_distance_;
  double goal_tolerance_;
  double linear_speed_;
  double max_angular_speed_;
};

}

#endif
