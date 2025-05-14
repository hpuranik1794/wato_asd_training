#include <chrono>
#include <memory>
#include <cmath>
 
#include "costmap_node.hpp"
using std::placeholders::_1;
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidar_subscribe, this, _1));
  costmap_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  // shift origin
  x_grid = (int)((x - origin_x)/resolution);
  y_grid = (int)((y - origin_y)/resolution);
}

void CostmapNode::inflateObstacles() {
  for (int i = 0; i < GRID_SIZE ; ++i) { // width
    for (int j = 0; j < GRID_SIZE; ++j) { // height
      if (occupancy_grid[i][j] == max_cost) {
        for (int x_inf = i - inf_radius; x_inf < i + inf_radius; ++x_inf) {
          for (int y_inf = j - inf_radius; y_inf < j + inf_radius; ++y_inf) {
            if (x_inf >= 0 && x_inf < GRID_SIZE && y_inf >= 0 && y_inf < GRID_SIZE) {
              double ed = sqrt(std::pow(((i - x_inf)), 2) + std::pow((j - y_inf), 2));
              if (ed <= inf_radius && occupancy_grid[x_inf][y_inf] < max_cost) {
                int cost = (int)(max_cost * (1 - (double)(ed/inf_radius)));
                if (occupancy_grid[x_inf][y_inf] < cost) {
                  occupancy_grid[x_inf][y_inf] = cost;
                }
              }           
            }
          }
        }
      }
    }
  }
}

void CostmapNode::initializeCostmap() {
  occupancy_grid = std::vector<std::vector<int>>(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));
}

void CostmapNode::lidar_subscribe(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      // Calculate grid coordinates
      int x_grid = 0, y_grid = 0;
      convertToGrid(range, angle, x_grid, y_grid);

      // mark obstacle in occupancy grid
      if (x_grid >= 0 && x_grid < GRID_SIZE && y_grid >= 0 && y_grid < GRID_SIZE) {
        occupancy_grid[y_grid][x_grid] = max_cost;
      }
    }
  }

  inflateObstacles();

  // flatten grid and publish
  costmap_grid.header = scan->header;

  costmap_grid.info.resolution = resolution;
  costmap_grid.info.origin.position.x = origin_x;
  costmap_grid.info.origin.position.y = origin_y;
  costmap_grid.info.width = GRID_SIZE;
  costmap_grid.info.height = GRID_SIZE;
  costmap_grid.info.origin.orientation.w = 1.0;

  costmap_grid.data.assign(GRID_SIZE * GRID_SIZE, -1);

  for (int i = 0; i < GRID_SIZE; ++i) {  // height
    for (int j = 0; j < GRID_SIZE; ++j) {  // width
      int cost = occupancy_grid[i][j];
      if (cost < 0) {
        cost = 0;
      } else if (cost > max_cost) {
        cost = max_cost;
      }
      costmap_grid.data[i * GRID_SIZE + j] = cost; // y * width + x
    }
  }

  occupancy_grid.clear();
  occupancy_grid.resize(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));

  costmap_grid_pub->publish(costmap_grid);
  // RCLCPP_INFO(this->get_logger(), "DONE: Published to /costmap");
}
  

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
