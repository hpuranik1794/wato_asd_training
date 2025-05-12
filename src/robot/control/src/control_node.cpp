#include "control_node.hpp"
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>
using std::placeholders::_1;

ControlNode::ControlNode(): Node("control") {
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/path", 10, [this](nav_msgs::msg::Path::SharedPtr msg){ path_ = msg; });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 20, [this](nav_msgs::msg::Odometry::SharedPtr msg){ odom_ = msg; });

  cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = create_wall_timer(std::chrono::milliseconds(100),
          std::bind(&ControlNode::controlLoop, this));       // 10 Hz
  }

  /* ---------- timer callback ---------- */
  void ControlNode::controlLoop()
  {
  if (!path_ || path_->poses.empty() || !odom_) return;

  const auto& robot    = odom_->pose.pose.position;
  const auto& goal     = path_->poses.back().pose.position;
  if (dist(robot.x, robot.y, goal.x, goal.y) < kGoalTol) {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
    return;
  }

  /* ---- pick lookahead point ---- */
  geometry_msgs::msg::PoseStamped target = path_->poses.back();   // fallback
  for (const auto& pose : path_->poses) {
    if (dist(robot.x, robot.y,
            pose.pose.position.x, pose.pose.position.y) >= kLookahead) {
      target = pose;
      break;
    }
  }

  /* ---- transform target to robot frame ---- */
  double yaw = yawFromQuat(odom_->pose.pose.orientation);
  double dx  = target.pose.position.x - robot.x;
  double dy  = target.pose.position.y - robot.y;

  double xr =  dx*std::cos(-yaw) - dy*std::sin(-yaw);
  double yr =  dx*std::sin(-yaw) + dy*std::cos(-yaw);

  /* ---- pure‑pursuit curvature ---- */
  double curvature = 2.0 * yr / (kLookahead * kLookahead);
  double ang = std::clamp(curvature * kCruise, -kMaxAng, kMaxAng);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = kCruise;
  cmd.angular.z = ang;

  cmd_pub_->publish(cmd);

}

/* ---------- helpers ---------- */
double ControlNode::dist(double x1,double y1,double x2,double y2)
{
  return std::hypot(x1 - x2, y1 - y2);
}
double ControlNode::yawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0*(q.z*q.w + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
