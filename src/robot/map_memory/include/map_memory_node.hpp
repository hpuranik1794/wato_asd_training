#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void listener(const nav_msgs::msg::Odometry::SharedPtr msg) const;

  private:
    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;
};

#endif 
