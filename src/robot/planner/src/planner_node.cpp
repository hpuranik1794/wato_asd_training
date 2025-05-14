#include "planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode() : Node("planner") {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

bool PlannerNode::worldToGrid(double wx, double wy, CellIndex& out) const {
  const auto& info = current_map_.info;
  double dx = wx - info.origin.position.x;
  double dy = wy - info.origin.position.y;
  int gx = static_cast<int>(std::floor(dx / info.resolution));
  int gy = static_cast<int>(std::floor(dy / info.resolution));
  if (gx < 0 || gy < 0 || gx >= static_cast<int>(info.width) || gy >= static_cast<int>(info.height)) {
    return false;
  }
  out = CellIndex(gx, gy);
  return true;
}

geometry_msgs::msg::PoseStamped PlannerNode::gridToPose(const CellIndex& c) const {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "sim_world";
  p.pose.position.x = current_map_.info.origin.position.x + (c.x + 0.5) * current_map_.info.resolution;
  p.pose.position.y = current_map_.info.origin.position.y + (c.y + 0.5) * current_map_.info.resolution;
  p.pose.orientation.w = 1.0;
  return p;
}

bool PlannerNode::isFree(CellIndex& g) const {
  int gx = g.x, gy = g.y;
  int idx = gy * static_cast<int>(current_map_.info.width) + gx;  // y * width + x
  return current_map_.data[idx] >= 0 && current_map_.data[idx] < 25;     // free or unknown but not lethal
}

double PlannerNode::heuristic(const CellIndex& a, const CellIndex& b) const {
  // diagonals allowed
  double dx = std::abs(a.x - b.x);
  double dy = std::abs(a.y - b.y);
  return std::max(dx, dy) + (std::sqrt(2) - 1) * std::min(dx, dy);
}


void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;

  if (goalReached()) {
    RCLCPP_INFO(get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
  } else {
    RCLCPP_INFO(get_logger(), "Replanning due to timeout or progress...");
    planPath();
  }
}

bool PlannerNode::goalReached() {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5;      // 0.5‑m tolerance
}

void PlannerNode::planPath() {
  if (state_ == State::WAITING_FOR_GOAL || current_map_.data.empty()) {
    RCLCPP_WARN(get_logger(), "Cannot plan path: missing map or goal");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "sim_world";

  // A* algorithm
  CellIndex start, goal;
  if (!worldToGrid(robot_pose_.position.x, robot_pose_.position.y, start) ||
      !worldToGrid(goal_.point.x, goal_.point.y, goal)) {
    RCLCPP_ERROR(get_logger(), "Start or goal outside map bounds");
    return;
  }

  // if (!isFree(goal)) {
  //   RCLCPP_WARN(get_logger(), "Goal in obstacle");
  //   return;
  // }

  using PQ = std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF>;
  PQ open;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start, goal));

  const std::vector<CellIndex> dirs = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}
  }; // 8 directions (4 cardinal + 4 diagonal)

  bool success = false;
  while (!open.empty()) {
    CellIndex current = open.top().index;
    open.pop();
    if (current == goal) { 
      success = true; 
      break;
    }

    for (const auto& d : dirs) {
      CellIndex neighbour{current.x + d.x, current.y + d.y};
      if (!isFree(neighbour)) continue;

      double tentative = g_score[current] + heuristic(neighbour, current);
      if (!g_score.count(neighbour) || tentative < g_score[neighbour]) {
        came_from[neighbour] = current;
        g_score[neighbour] = tentative;
        double f = tentative + heuristic(neighbour, goal);
        open.emplace(neighbour, f);
      }
    }
  }

  if (!success) {
    RCLCPP_WARN(get_logger(), "A*: No path found");
    return;
  }

  // reconstruct path
  CellIndex cur = goal;
  while (cur != start) {
    if (came_from.find(cur) == came_from.end()) {
      RCLCPP_WARN(get_logger(), "A*: No path found");
      return;
    }
    path.poses.emplace_back(gridToPose(cur));
    cur = came_from[cur];
  }

  // add start position
  path.poses.emplace_back(gridToPose(start));
  std::reverse(path.poses.begin(), path.poses.end());

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
