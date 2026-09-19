#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger & logger, int obstacle_threshold)
: logger_(logger),
  obstacle_threshold_(std::max(1, obstacle_threshold))
{
}

std::vector<geometry_msgs::msg::PoseStamped> PlannerCore::plan(
  const nav_msgs::msg::OccupancyGrid & map,
  double start_x,
  double start_y,
  double goal_x,
  double goal_y) const
{
  const auto width = static_cast<std::size_t>(map.info.width);
  const auto height = static_cast<std::size_t>(map.info.height);
  if (width == 0U || height == 0U || map.info.resolution <= 0.0F ||
    map.data.size() < width * height)
  {
    RCLCPP_WARN(logger_, "Cannot plan: map dimensions, resolution, or data are invalid");
    return {};
  }

  const CellIndex requested_start = world_to_grid(map, start_x, start_y);
  const CellIndex requested_goal = world_to_grid(map, goal_x, goal_y);
  const auto start = nearest_traversable(map, requested_start);
  const auto goal = nearest_traversable(map, requested_goal);
  if (!start || !goal) {
    RCLCPP_WARN(logger_, "Cannot plan: no traversable start or goal cell exists");
    return {};
  }

  if (*start == *goal) {
    return {cell_to_pose(map, *start)};
  }

  const std::size_t cell_count = width * height;
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> g_score(cell_count, infinity);
  std::vector<CellIndex> parents(cell_count);
  std::vector<bool> has_parent(cell_count, false);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

  const std::size_t start_index = flat_index(map, *start);
  g_score[start_index] = 0.0;
  open_set.emplace(*start, heuristic(*start, *goal));

  constexpr std::array<int, 8> dx{{-1, 0, 1, -1, 1, -1, 0, 1}};
  constexpr std::array<int, 8> dy{{-1, -1, -1, 0, 0, 1, 1, 1}};
  bool found = false;

  while (!open_set.empty()) {
    const CellIndex current = open_set.top().index;
    open_set.pop();
    const std::size_t current_index = flat_index(map, current);
    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = true;

    if (current == *goal) {
      found = true;
      break;
    }

    for (std::size_t direction = 0; direction < dx.size(); ++direction) {
      const CellIndex neighbor(
        current.x + dx[direction],
        current.y + dy[direction]);
      if (!in_bounds(map, neighbor) || !traversable(map, neighbor)) {
        continue;
      }

      const bool diagonal = dx[direction] != 0 && dy[direction] != 0;
      if (diagonal) {
        const CellIndex horizontal(current.x + dx[direction], current.y);
        const CellIndex vertical(current.x, current.y + dy[direction]);
        if (!traversable(map, horizontal) || !traversable(map, vertical)) {
          continue;
        }
      }

      const std::size_t neighbor_index = flat_index(map, neighbor);
      if (closed[neighbor_index]) {
        continue;
      }

      const double step_cost = diagonal ? std::sqrt(2.0) : 1.0;
      const double tentative_g = g_score[current_index] + step_cost;
      if (tentative_g >= g_score[neighbor_index]) {
        continue;
      }

      parents[neighbor_index] = current;
      has_parent[neighbor_index] = true;
      g_score[neighbor_index] = tentative_g;
      open_set.emplace(
        neighbor,
        tentative_g + heuristic(neighbor, *goal));
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* could not find a path to the requested goal");
    return {};
  }

  std::vector<CellIndex> cells;
  CellIndex current = *goal;
  while (current != *start) {
    cells.push_back(current);
    const std::size_t current_index = flat_index(map, current);
    if (!has_parent[current_index]) {
      RCLCPP_WARN(logger_, "A* path reconstruction failed");
      return {};
    }
    current = parents[current_index];
  }
  cells.push_back(*start);
  std::reverse(cells.begin(), cells.end());

  std::vector<geometry_msgs::msg::PoseStamped> path;
  path.reserve(cells.size());
  for (const CellIndex & cell : cells) {
    path.push_back(cell_to_pose(map, cell));
  }

  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const double yaw = std::atan2(
      path[index + 1].pose.position.y - path[index].pose.position.y,
      path[index + 1].pose.position.x - path[index].pose.position.x);
    path[index].pose.orientation.z = std::sin(yaw / 2.0);
    path[index].pose.orientation.w = std::cos(yaw / 2.0);
  }
  if (path.size() > 1U) {
    path.back().pose.orientation = path[path.size() - 2U].pose.orientation;
  }

  return path;
}

bool PlannerCore::in_bounds(
  const nav_msgs::msg::OccupancyGrid & map,
  const CellIndex & cell) const
{
  return cell.x >= 0 && cell.y >= 0 &&
    cell.x < static_cast<int>(map.info.width) &&
    cell.y < static_cast<int>(map.info.height);
}

bool PlannerCore::traversable(
  const nav_msgs::msg::OccupancyGrid & map,
  const CellIndex & cell) const
{
  if (!in_bounds(map, cell)) {
    return false;
  }
  return map.data[flat_index(map, cell)] < obstacle_threshold_;
}

std::optional<CellIndex> PlannerCore::nearest_traversable(
  const nav_msgs::msg::OccupancyGrid & map,
  CellIndex desired) const
{
  if (map.info.width == 0U || map.info.height == 0U) {
    return std::nullopt;
  }

  desired.x = std::clamp(desired.x, 0, static_cast<int>(map.info.width) - 1);
  desired.y = std::clamp(desired.y, 0, static_cast<int>(map.info.height) - 1);

  std::queue<CellIndex> pending;
  std::vector<bool> visited(
    static_cast<std::size_t>(map.info.width) *
    static_cast<std::size_t>(map.info.height), false);
  pending.push(desired);
  visited[flat_index(map, desired)] = true;

  constexpr std::array<int, 8> dx{{-1, 0, 1, -1, 1, -1, 0, 1}};
  constexpr std::array<int, 8> dy{{-1, -1, -1, 0, 0, 1, 1, 1}};
  while (!pending.empty()) {
    const CellIndex current = pending.front();
    pending.pop();
    if (traversable(map, current)) {
      return current;
    }

    for (std::size_t direction = 0; direction < dx.size(); ++direction) {
      const CellIndex neighbor(
        current.x + dx[direction],
        current.y + dy[direction]);
      if (in_bounds(map, neighbor)) {
        const std::size_t index = flat_index(map, neighbor);
        if (!visited[index]) {
          visited[index] = true;
          pending.push(neighbor);
        }
      }
    }
  }

  return std::nullopt;
}

CellIndex PlannerCore::world_to_grid(
  const nav_msgs::msg::OccupancyGrid & map,
  double x,
  double y) const
{
  return CellIndex(
    static_cast<int>(std::floor(
      (x - map.info.origin.position.x) / map.info.resolution)),
    static_cast<int>(std::floor(
      (y - map.info.origin.position.y) / map.info.resolution)));
}

geometry_msgs::msg::PoseStamped PlannerCore::cell_to_pose(
  const nav_msgs::msg::OccupancyGrid & map,
  const CellIndex & cell) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = map.header.frame_id;
  pose.pose.position.x =
    map.info.origin.position.x +
    (static_cast<double>(cell.x) + 0.5) * map.info.resolution;
  pose.pose.position.y =
    map.info.origin.position.y +
    (static_cast<double>(cell.y) + 0.5) * map.info.resolution;
  pose.pose.orientation.w = 1.0;
  return pose;
}

double PlannerCore::heuristic(const CellIndex & from, const CellIndex & to)
{
  return std::hypot(
    static_cast<double>(from.x - to.x),
    static_cast<double>(from.y - to.y));
}

std::size_t PlannerCore::flat_index(
  const nav_msgs::msg::OccupancyGrid & map,
  const CellIndex & cell)
{
  return static_cast<std::size_t>(cell.y) *
    static_cast<std::size_t>(map.info.width) +
    static_cast<std::size_t>(cell.x);
}

}
