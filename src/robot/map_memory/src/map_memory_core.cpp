#include <algorithm>
#include <cmath>
#include <cstdint>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(
  const rclcpp::Logger& logger,
  double resolution,
  int width,
  int height)
: logger_(logger),
  resolution_(resolution > 0.0 ? resolution : 0.1),
  width_(std::max(1, width)),
  height_(std::max(1, height)),
  origin_x_(-static_cast<double>(width_) * resolution_ / 2.0),
  origin_y_(-static_cast<double>(height_) * resolution_ / 2.0),
  grid_(
    static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_),
    static_cast<int8_t>(-1))
{
}

void MapMemoryCore::integrate(
  const nav_msgs::msg::OccupancyGrid & costmap,
  double robot_x,
  double robot_y,
  double robot_yaw)
{
  const int local_width = static_cast<int>(costmap.info.width);
  const int local_height = static_cast<int>(costmap.info.height);
  const double local_res = static_cast<double>(costmap.info.resolution);
  const double local_origin_x = costmap.info.origin.position.x;
  const double local_origin_y = costmap.info.origin.position.y;

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  for (int local_y = 0; local_y < local_height; ++local_y) {
    for (int local_x = 0; local_x < local_width; ++local_x) {
      const std::size_t local_index =
        static_cast<std::size_t>(local_y) *
          static_cast<std::size_t>(local_width) +
        static_cast<std::size_t>(local_x);
      const int8_t cost = costmap.data[local_index];

      if (cost <= 0) {
        continue;
      }

      const double x_local =
        local_origin_x + (static_cast<double>(local_x) + 0.5) * local_res;
      const double y_local =
        local_origin_y + (static_cast<double>(local_y) + 0.5) * local_res;

      const double x_world =
        robot_x + x_local * cos_yaw - y_local * sin_yaw;
      const double y_world =
        robot_y + x_local * sin_yaw + y_local * cos_yaw;

      int grid_x = 0;
      int grid_y = 0;
      if (!world_to_grid(x_world, y_world, grid_x, grid_y)) {
        continue;
      }

      const std::size_t global_index =
        static_cast<std::size_t>(grid_y) *
          static_cast<std::size_t>(width_) +
        static_cast<std::size_t>(grid_x);
      grid_[global_index] = cost;
    }
  }
}

bool MapMemoryCore::world_to_grid(
  double x,
  double y,
  int & grid_x,
  int & grid_y) const
{
  grid_x = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  grid_y = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::to_message() const
{
  nav_msgs::msg::OccupancyGrid message;
  message.header.frame_id = "sim_world";
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
