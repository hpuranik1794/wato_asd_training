#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
    // void initializeCostmap();
    void subscribeMessage(const std_msgs::msg::String::SharedPtr msg) const;
    void lidar_subscribe(sensor_msgs::msg::LaserScan::SharedPtr scan);

 
  private:
    robot::CostmapCore costmap_;
    double origin_x = -20; double origin_y = -20;
    int GRID_SIZE = 401;
    double max_cost = 100;
    int inf_radius = 10;
    double resolution = 0.1;
    std::vector<std::vector<double>> occupancy_grid;

    void convert_to_grid(double range, double angle, int& x_grid, int& y_grid);

    
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_grid_pub;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
#endif 
