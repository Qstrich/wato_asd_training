#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
: Node("planner_node"),
  planner_(
    this->get_logger(),
    this->declare_parameter<int>("obstacle_threshold", 1)),
  state_(State::WAITING_FOR_GOAL),
  have_map_(false),
  have_goal_(false),
  have_odom_(false),
  have_path_(false),
  goal_tolerance_(
    this->declare_parameter<double>("goal_tolerance", 0.5)),
  replan_timeout_(
    this->declare_parameter<double>("replan_timeout", 5.0)),
  timer_period_ms_(
    this->declare_parameter<int>("timer_period_ms", 500)),
  last_plan_time_(std::chrono::steady_clock::now())
{
  goal_tolerance_ = std::max(0.01, goal_tolerance_);
  replan_timeout_ = std::max(0.1, replan_timeout_);
  timer_period_ms_ = std::max(1, timer_period_ms_);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    10,
    std::bind(
      &PlannerNode::map_callback,
      this,
      std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point",
    10,
    std::bind(
      &PlannerNode::goal_callback,
      this,
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(
      &PlannerNode::odom_callback,
      this,
      std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_period_ms_),
    std::bind(&PlannerNode::timer_callback, this));
}

void PlannerNode::map_callback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  have_map_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    plan_path();
  }
}

void PlannerNode::goal_callback(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  have_goal_ = true;
  have_path_ = false;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  plan_path();
}

void PlannerNode::odom_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odometry_ = *msg;
  have_odom_ = true;
}

void PlannerNode::timer_callback()
{
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL ||
    !have_goal_ || !have_odom_)
  {
    return;
  }

  if (goal_reached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    state_ = State::WAITING_FOR_GOAL;
    have_path_ = false;
    publish_empty_path();
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double elapsed =
    std::chrono::duration<double>(now - last_plan_time_).count();
  if (elapsed >= replan_timeout_) {
    plan_path();
  }
}

void PlannerNode::plan_path()
{
  if (!have_map_ || !have_goal_ || !have_odom_) {
    return;
  }

  const auto now = this->now();
  const std::string frame_id =
    current_map_.header.frame_id.empty() ?
    std::string("sim_world") : current_map_.header.frame_id;
  const auto poses = planner_.plan(
    current_map_,
    odometry_.pose.pose.position.x,
    odometry_.pose.pose.position.y,
    goal_.point.x,
    goal_.point.y);

  nav_msgs::msg::Path path;
  path.header.stamp = now;
  path.header.frame_id = frame_id;
  path.poses = poses;
  for (auto & pose : path.poses) {
    pose.header.stamp = now;
    pose.header.frame_id = frame_id;
  }
  path_pub_->publish(path);
  have_path_ = !path.poses.empty();
  last_plan_time_ = std::chrono::steady_clock::now();

  if (!have_path_) {
    RCLCPP_WARN(this->get_logger(), "No path found for the current goal");
  }
}

void PlannerNode::publish_empty_path()
{
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id =
    current_map_.header.frame_id.empty() ?
    std::string("sim_world") : current_map_.header.frame_id;
  path_pub_->publish(path);
}

bool PlannerNode::goal_reached() const
{
  const double dx =
    goal_.point.x - odometry_.pose.pose.position.x;
  const double dy =
    goal_.point.y - odometry_.pose.pose.position.y;
  return std::hypot(dx, dy) <= goal_tolerance_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
