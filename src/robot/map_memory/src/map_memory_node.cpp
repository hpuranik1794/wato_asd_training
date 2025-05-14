#include <chrono>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Initialize subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
  
  // Initialize global map
  global_map_.info.resolution = 0.1;
  global_map_.info.origin.position.x = -15.0;
  global_map_.info.origin.position.y = -15.0;
  global_map_.info.width = GLOBALMAP_GRID_SIZE;
  global_map_.info.height = GLOBALMAP_GRID_SIZE;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.resize(GLOBALMAP_GRID_SIZE * GLOBALMAP_GRID_SIZE, 0);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "RECEIVED NEW COSTMAP");
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  // RCLCPP_INFO(this->get_logger(), "Robot (x): '%f'", x);
  // RCLCPP_INFO(this->get_logger(), "Robot (y): '%f'", y);

  ox = msg->pose.pose.orientation.x;
  oy = msg->pose.pose.orientation.y;
  oz = msg->pose.pose.orientation.z;
  ow = msg->pose.pose.orientation.w;
  
  yaw = std::atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oy * oy + oz * oz));
  // RCLCPP_INFO(this->get_logger(), "Robot (yaw): '%f'", yaw);

  // Compute distance traveled
  double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
  if (distance >= distance_threshold) {
    last_x = x;
    last_y = y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    integrateCostmap();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
    costmap_updated_ = false;
  }
}

void MapMemoryNode::integrateCostmap() {
  RCLCPP_INFO(this->get_logger(), "UPDATING GLOBAL MAP...");
  int costmap_width = latest_costmap_.info.width, costmap_height = latest_costmap_.info.height;

  // Initialize 2D map
  globalmap_2d = std::vector<std::vector<int>>(GLOBALMAP_GRID_SIZE, std::vector<int>(GLOBALMAP_GRID_SIZE, -1));

  for (int i=0; i<costmap_height; ++i) {
    for (int j=0; j<costmap_width; ++j) {
      // calculate index wrt the robot's position in costmap (j - 20, i - 20)
      double rel_x = j * latest_costmap_.info.resolution + latest_costmap_.info.origin.position.x;
      double rel_y = i * latest_costmap_.info.resolution + latest_costmap_.info.origin.position.y;

      // calculate index wrt the robot's orientation in grid
      double rel_x_rotated = rel_x * std::cos(yaw) - rel_y * std::sin(yaw);
      double rel_y_rotated = rel_x * std::sin(yaw) + rel_y * std::cos(yaw);
      rel_x = rel_x_rotated + last_x;
      rel_y = rel_y_rotated + last_y;

      // re-shift the origin (adj_j - 20 + 15, adj_i - 20 + 15)
      int abs_x = (int)((rel_x - global_map_.info.origin.position.x)/global_map_.info.resolution);
      int abs_y = (int)((rel_y - global_map_.info.origin.position.y)/global_map_.info.resolution);
      
      // check if indices are out of bounds
      if (abs_x < 0 || abs_x >= GLOBALMAP_GRID_SIZE || abs_y < 0 || abs_y >= GLOBALMAP_GRID_SIZE) {
        continue;
      }
      
      // update based on priorities
      globalmap_2d[abs_y][abs_x] = (int)(latest_costmap_.data[i * latest_costmap_.info.width + j] * 0.75 + globalmap_2d[abs_y][abs_x] * 0.25);
    }
  }

  global_map_.header.frame_id = "sim_world";
  global_map_.header.stamp = this->now();

  // flatten 2D global map
  for (int i=0; i<GLOBALMAP_GRID_SIZE; ++i) {
    for (int j=0; j<GLOBALMAP_GRID_SIZE; ++j) {
      int idx = i * GLOBALMAP_GRID_SIZE + j; // y * width + x
      if (globalmap_2d[i][j] > -1 && global_map_.data[idx] < globalmap_2d[i][j]) {
        global_map_.data[idx] = globalmap_2d[i][j];
      }
    }
  }
  RCLCPP_INFO(this->get_logger(), "DONE: Updated global map");
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
