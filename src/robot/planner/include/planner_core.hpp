#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex & other) const
  {
    return x == other.x && y == other.y;
  }

  bool operator!=(const CellIndex & other) const
  {
    return !(*this == other);
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex & index) const
  {
    return std::hash<int>()(index.x) ^ (std::hash<int>()(index.y) << 1);
  }
};

struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(CellIndex cell, double score)
  : index(cell), f_score(score) {}
};

struct CompareF
{
  bool operator()(const AStarNode & lhs, const AStarNode & rhs) const
  {
    return lhs.f_score > rhs.f_score;
  }
};

class PlannerCore
{
public:
  explicit PlannerCore(const rclcpp::Logger & logger, int obstacle_threshold = 1);

  std::vector<geometry_msgs::msg::PoseStamped> plan(
    const nav_msgs::msg::OccupancyGrid & map,
    double start_x,
    double start_y,
    double goal_x,
    double goal_y) const;

private:
  bool in_bounds(const nav_msgs::msg::OccupancyGrid & map, const CellIndex & cell) const;
  bool traversable(
    const nav_msgs::msg::OccupancyGrid & map,
    const CellIndex & cell) const;
  std::optional<CellIndex> nearest_traversable(
    const nav_msgs::msg::OccupancyGrid & map,
    CellIndex desired) const;
  CellIndex world_to_grid(
    const nav_msgs::msg::OccupancyGrid & map,
    double x,
    double y) const;
  geometry_msgs::msg::PoseStamped cell_to_pose(
    const nav_msgs::msg::OccupancyGrid & map,
    const CellIndex & cell) const;
  static double heuristic(const CellIndex & from, const CellIndex & to);
  static std::size_t flat_index(
    const nav_msgs::msg::OccupancyGrid & map,
    const CellIndex & cell);

  rclcpp::Logger logger_;
  int obstacle_threshold_;
};

}

#endif
