#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"

#include <cmath>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    /* ---------- parameters ---------- */
    static constexpr double kLookahead = 1.0;        // [m]
    static constexpr double kGoalTol   = 0.15;       // [m]
    static constexpr double kCruise    = 0.40;       // [m/s]
    static constexpr double kMaxAng    = 1.50;       // [rad/s]
  
    /* ---------- ROS I/O ---------- */
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr      path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   cmd_pub_;
    rclcpp::TimerBase::SharedPtr                               timer_;
  
    /* ---------- cached data ---------- */
    nav_msgs::msg::Path::SharedPtr      path_;
    nav_msgs::msg::Odometry::SharedPtr  odom_;
  
    /* ---------- helpers ---------- */
    void controlLoop();                               // timer callback
    static double dist(double x1,double y1,double x2,double y2);
    static double yawFromQuat(const geometry_msgs::msg::Quaternion& q);   
};

#endif
