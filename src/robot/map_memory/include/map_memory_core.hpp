#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class MapMemoryCore {
public:
  MapMemoryCore(
    const rclcpp::Logger& logger,
    double resolution,
    int width,
    int height);

  void integrate(
    const nav_msgs::msg::OccupancyGrid & costmap,
    double robot_x,
    double robot_y,
    double robot_yaw);

  nav_msgs::msg::OccupancyGrid to_message() const;

private:
  bool world_to_grid(double x, double y, int & grid_x, int & grid_y) const;

  rclcpp::Logger logger_;
  double resolution_;
  int width_;
  int height_;
  double origin_x_;
  double origin_y_;
  std::vector<int8_t> grid_;
};

}

#endif
