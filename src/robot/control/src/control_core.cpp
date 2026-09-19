#include <algorithm>
#include <cmath>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(
  const rclcpp::Logger & logger,
  double lookahead_distance,
  double goal_tolerance,
  double linear_speed,
  double max_angular_speed)
: logger_(logger),
  lookahead_distance_(std::max(1e-3, lookahead_distance)),
  goal_tolerance_(std::max(0.0, goal_tolerance)),
  linear_speed_(std::max(0.0, linear_speed)),
  max_angular_speed_(std::max(0.0, max_angular_speed))
{
}

ControlResult ControlCore::compute_command(
  const nav_msgs::msg::Path & path,
  const nav_msgs::msg::Odometry & odometry) const
{
  ControlResult result;
  result.stop = true;
  if (path.poses.empty()) {
    return result;
  }

  const geometry_msgs::msg::Point & robot_position =
    odometry.pose.pose.position;
  const geometry_msgs::msg::Point & final_position =
    path.poses.back().pose.position;
  if (distance(robot_position, final_position) <= goal_tolerance_) {
    return result;
  }

  const geometry_msgs::msg::PoseStamped * target = &path.poses.back();
  for (const auto & pose : path.poses) {
    if (distance(robot_position, pose.pose.position) >= lookahead_distance_) {
      target = &pose;
      break;
    }
  }

  const double yaw = yaw_from_quaternion(odometry.pose.pose.orientation);
  const double dx = target->pose.position.x - robot_position.x;
  const double dy = target->pose.position.y - robot_position.y;
  const double x_robot = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double y_robot = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const double target_distance = std::hypot(x_robot, y_robot);
  if (!std::isfinite(target_distance) || target_distance < 1e-3) {
    return result;
  }

  const double curvature =
    2.0 * y_robot / (target_distance * target_distance);
  result.command.linear.x = linear_speed_;
  result.command.angular.z = std::clamp(
    linear_speed_ * curvature,
    -max_angular_speed_,
    max_angular_speed_);
  result.stop = false;
  return result;
}

double ControlCore::distance(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

double ControlCore::yaw_from_quaternion(
  const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

}
