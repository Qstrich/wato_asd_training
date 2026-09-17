#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode()
: Node("costmap_node")
{
  const double resolution = this->declare_parameter("resolution", 0.1);
  const int width = this->declare_parameter("width", 300);
  const int height = this->declare_parameter("height", 300);
  const double inflation_radius =
    this->declare_parameter("inflation_radius", 1.0);
  const int max_cost = this->declare_parameter("max_cost", 100);

  costmap_ = std::make_unique<robot::CostmapCore>(
    this->get_logger(),
    resolution,
    width,
    height,
    inflation_radius,
    max_cost);

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    rclcpp::SensorDataQoS(),
    std::bind(
      &CostmapNode::laser_callback,
      this,
      std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    10);
}

void CostmapNode::laser_callback(
  const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  costmap_->update(*scan);
  costmap_pub_->publish(costmap_->to_message(*scan));
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}