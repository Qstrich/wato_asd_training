#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
public:
  CostmapCore(
    const rclcpp::Logger& logger,
    double resolution,
    int width,
    int height,
    double inflation_radius,
    int max_cost);

  void update(const sensor_msgs::msg::LaserScan & scan);
  nav_msgs::msg::OccupancyGrid to_message(
    const sensor_msgs::msg::LaserScan & scan) const;

private:
  void inflate_obstacles(const std::vector<std::pair<int, int>> & obstacles);
  bool world_to_grid(double x, double y, int & grid_x, int & grid_y) const;

  rclcpp::Logger logger_;
  double resolution_;
  int width_;
  int height_;
  double inflation_radius_;
  int max_cost_;
  double origin_x_;
  double origin_y_;
  std::vector<int8_t> grid_;
};

}  

#endif  