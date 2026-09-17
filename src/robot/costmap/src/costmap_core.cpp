#include <algorithm>
#include <cmath>
#include <utility>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(
  const rclcpp::Logger& logger,
  double resolution,
  int width,
  int height,
  double inflation_radius,
  int max_cost)
: logger_(logger),
  resolution_(resolution > 0.0 ? resolution : 0.1),
  width_(std::max(1, width)),
  height_(std::max(1, height)),
  inflation_radius_(std::max(0.0, inflation_radius)),
  max_cost_(std::clamp(max_cost, 0, 100)),
  origin_x_(-static_cast<double>(width_) * resolution_ / 2.0),
  origin_y_(-static_cast<double>(height_) * resolution_ / 2.0),
  grid_(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), 0)
{
}

void CostmapCore::update(const sensor_msgs::msg::LaserScan & scan)
{
  grid_.assign(
    static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), 0);

  std::vector<std::pair<int, int>> obstacles;
  obstacles.reserve(scan.ranges.size());

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range <= scan.range_min || range >= scan.range_max) {
      continue;
    }

    const double angle =
      static_cast<double>(scan.angle_min) +
      static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    const double x = static_cast<double>(range) * std::cos(angle);
    const double y = static_cast<double>(range) * std::sin(angle);

    int grid_x;
    int grid_y;
    if (world_to_grid(x, y, grid_x, grid_y)) {
      const std::size_t cell =
        static_cast<std::size_t>(grid_y) * static_cast<std::size_t>(width_) +
        static_cast<std::size_t>(grid_x);
      grid_[cell] = static_cast<int8_t>(max_cost_);
      obstacles.emplace_back(grid_x, grid_y);
    }
  }

  inflate_obstacles(obstacles);
}

bool CostmapCore::world_to_grid(
  double x,
  double y,
  int & grid_x,
  int & grid_y) const
{
  grid_x = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  grid_y = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
}

void CostmapCore::inflate_obstacles(
  const std::vector<std::pair<int, int>> & obstacles)
{
  if (inflation_radius_ <= 0.0 || max_cost_ <= 0) {
    return;
  }

  const int cell_radius =
    static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  for (const auto & obstacle : obstacles) {
    for (int offset_y = -cell_radius; offset_y <= cell_radius; ++offset_y) {
      for (int offset_x = -cell_radius; offset_x <= cell_radius; ++offset_x) {
        const int grid_x = obstacle.first + offset_x;
        const int grid_y = obstacle.second + offset_y;
        if (grid_x < 0 || grid_x >= width_ || grid_y < 0 || grid_y >= height_) {
          continue;
        }

        const double distance = std::hypot(
          static_cast<double>(offset_x) * resolution_,
          static_cast<double>(offset_y) * resolution_);
        if (distance > inflation_radius_) {
          continue;
        }

        const double cost =
          static_cast<double>(max_cost_) *
          (1.0 - distance / inflation_radius_);
        const int inflated_cost = static_cast<int>(std::lround(cost));
        const std::size_t cell =
          static_cast<std::size_t>(grid_y) * static_cast<std::size_t>(width_) +
          static_cast<std::size_t>(grid_x);
        if (inflated_cost > grid_[cell]) {
          grid_[cell] = static_cast<int8_t>(inflated_cost);
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::to_message(
  const sensor_msgs::msg::LaserScan & scan) const
{
  nav_msgs::msg::OccupancyGrid message;
  message.header = scan.header;
  message.info.resolution = static_cast<float>(resolution_);
  message.info.width = static_cast<std::uint32_t>(width_);
  message.info.height = static_cast<std::uint32_t>(height_);
  message.info.origin.position.x = origin_x_;
  message.info.origin.position.y = origin_y_;
  message.info.origin.orientation.w = 1.0;
  message.data = grid_;
  return message;
}

}