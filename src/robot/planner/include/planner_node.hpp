#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  enum class State
  {
    WAITING_FOR_GOAL,
    WAITING_FOR_ROBOT_TO_REACH_GOAL
  };

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timer_callback();
  void plan_path();
  void publish_empty_path();
  bool goal_reached() const;

  robot::PlannerCore planner_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  nav_msgs::msg::Odometry odometry_;

  State state_;
  bool have_map_;
  bool have_goal_;
  bool have_odom_;
  bool have_path_;
  double goal_tolerance_;
  double replan_timeout_;
  int timer_period_ms_;
  std::chrono::steady_clock::time_point last_plan_time_;
};

#endif
