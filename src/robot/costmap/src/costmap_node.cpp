#include <chrono>
#include <memory>
#include <cmath>
 
#include "costmap_node.hpp"
using std::placeholders::_1;
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  occupancy_grid = std::vector<std::vector<double>>(401, std::vector<double>(401, 0));
  string_sub_ = this->create_subscription<std_msgs::msg::String>("/test_topic", 10, std::bind(&CostmapNode::subscribeMessage, this, _1));
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidar_subscribe, this, _1));
  costmap_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);


  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::subscribeMessage(const std_msgs::msg::String::SharedPtr msg) const {
  // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

void CostmapNode::convert_to_grid(double range, double angle, int& x_grid, int& y_grid) {
  x_grid = range * std::cos(angle);
  y_grid = range * std::sin(angle);
  x_grid = static_cast<int>((x_grid - origin_x)/resolution);
  y_grid = static_cast<int>((y_grid - origin_y)/resolution);
  // RCLCPP_INFO(this->get_logger(), "CTGGGGGGGGG");
}

void CostmapNode::lidar_subscribe(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
        // Calculate grid coordinates
        int x_grid, y_grid;
        convert_to_grid(range, angle, x_grid, y_grid);
        if (x_grid < 0 || x_grid >= GRID_SIZE || y_grid < 0 || y_grid >= GRID_SIZE) {
          continue;
        }

        occupancy_grid[x_grid][y_grid] = max_cost;
        for (int x_inflation = 0; x_inflation < GRID_SIZE; ++x_inflation) {
          for (int y_inflation = 0; y_inflation < GRID_SIZE; ++y_inflation) {
            double distance = std::sqrt(std::pow(x_inflation - x_grid, 2) + std::pow(y_inflation - y_grid, 2));
            double cost = max_cost * (1 - (distance/inf_radius));
            occupancy_grid[x_inflation][y_inflation] = std::max(occupancy_grid[x_inflation][y_inflation], cost);
          }
        }
    }
  }

  nav_msgs::msg::OccupancyGrid costmap_grid;
  // header
  costmap_grid.header.frame_id = "sim_world";
  costmap_grid.header.stamp = this->now();

  // info
  costmap_grid.info.resolution = resolution;
  costmap_grid.info.origin.position.x = origin_x;
  costmap_grid.info.origin.position.y = origin_y;
  costmap_grid.info.width = GRID_SIZE;
  costmap_grid.info.height = GRID_SIZE;

  costmap_grid.data.resize(GRID_SIZE * GRID_SIZE);

  for (int i = 0; i < GRID_SIZE; ++i) {
    for (int j = 0; j < GRID_SIZE; ++j) {
      costmap_grid.data[i * GRID_SIZE + j] = occupancy_grid[i][j];
    }
  }

  costmap_grid_pub->publish(costmap_grid);
  RCLCPP_INFO(this->get_logger(), "DONE: PUBLISHED!");
}
  

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}