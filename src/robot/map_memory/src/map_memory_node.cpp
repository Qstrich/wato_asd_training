#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory_node"),
  distance_threshold_(1.5),
  last_x_(0.0),
  last_y_(0.0),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_yaw_(0.0),
  have_pose_(false),
  costmap_updated_(false),
  should_update_map_(false)
{
  const double resolution = this->declare_parameter("resolution", 0.1);
  const int width = this->declare_parameter("width", 400);
  const int height = this->declare_parameter("height", 400);
  distance_threshold_ = this->declare_parameter("distance_threshold", 1.5);
  const double update_frequency =
    this->declare_parameter("update_frequency", 1.0);

  map_memory_ = std::make_unique<robot::MapMemoryCore>(
    this->get_logger(),
    resolution,
    width,
    height);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    10,
    std::bind(
      &MapMemoryNode::costmap_callback,
      this,
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(
      &MapMemoryNode::odom_callback,
      this,
      std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  const double frequency = std::max(update_frequency, 0.001);
  const int period_ms = static_cast<int>(std::lround(1000.0 / frequency));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&MapMemoryNode::update_map, this));
}

void MapMemoryNode::costmap_callback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odom_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);

  if (!have_pose_) {
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    have_pose_ = true;
    should_update_map_ = true;
    return;
  }

  const double distance = std::hypot(robot_x_ - last_x_, robot_y_ - last_y_);
  if (distance >= distance_threshold_) {
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    should_update_map_ = true;
  }
}

void MapMemoryNode::update_map()
{
  if (!should_update_map_ || !costmap_updated_ || !have_pose_) {
    return;
  }

  map_memory_->integrate(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
  nav_msgs::msg::OccupancyGrid message = map_memory_->to_message();
  message.header.stamp = this->now();
  map_pub_->publish(message);
  should_update_map_ = false;
}

double MapMemoryNode::yaw_from_quaternion(
  const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
