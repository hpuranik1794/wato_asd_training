#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"

#define GRID_SIZE 400
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    void lidar_subscribe(sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    robot::CostmapCore costmap_;
    double origin_x = -20.0;
    double origin_y = -20.0;
    int max_cost = 100;
    int inf_radius = 10;
    double resolution = 0.1;
    std::vector<std::vector<int>> occupancy_grid;
    nav_msgs::msg::OccupancyGrid costmap_grid;

    void initializeCostmap();
    void inflateObstacles();
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);

    
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_grid_pub;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
#endif 
