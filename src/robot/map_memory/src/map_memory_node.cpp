#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::listener, this, _1));
}

void MapMemoryNode::listener(const nav_msgs::msg::Odometry::SharedPtr msg) const {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  RCLCPP_INFO(this->get_logger(), "Odometry x: '%f'", x);
  RCLCPP_INFO(this->get_logger(), "Odometry y: '%f'", y);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
