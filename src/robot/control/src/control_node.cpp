#include "control_node.hpp"
#include <algorithm>
using std::placeholders::_1;

ControlNode::ControlNode(): Node("control") {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](nav_msgs::msg::Path::SharedPtr msg){ current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 20, [this](nav_msgs::msg::Odometry::SharedPtr msg){ robot_odom_ = msg; });
    
  cmd_vel_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  if (!current_path_ || current_path_->poses.empty() || !robot_odom_) return;

  const auto robot = robot_odom_->pose.pose.position;
  const auto goal = current_path_->poses.back().pose.position;
  if (dist(robot, goal) < goal_tolerance) {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0;
    stop.angular.z = 0;
    RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot.");
    cmd_vel_pub_->publish(stop);
    return;
  }

  const auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    RCLCPP_WARN(this->get_logger(), "No lookahead point found!");
    return;
  }

  const auto cmd_vel = computeVelocity(*lookahead_point);

  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  const auto robot = robot_odom_->pose.pose.position;
  for (const auto& pose : current_path_->poses) {
    double d = dist(robot, pose.pose.position);
    if (d >= lookahead_dist) {
      return pose;
    }
  }
  return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  const auto robot = robot_odom_->pose.pose.position;
  geometry_msgs::msg::Twist cmd_vel;
  double yaw = extractYaw(robot_odom_->pose.pose.orientation);
  double dx  = target.pose.position.x - robot.x;
  double dy  = target.pose.position.y - robot.y;

  double angle = std::atan2(dy, dx) - yaw;

  // adjust angle
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2 * M_PI;
  }

  RCLCPP_INFO(this->get_logger(), "Speed: %f, Angle: %f", linear_speed, angle);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_speed;
  cmd.angular.z = angle;
  return cmd_vel;
}

double ControlNode::dist(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0*(q.z*q.w + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
