#pragma once
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <lwr_controllers/msg/pose_rpy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace ultrasound_trajectory {
template<typename T>
T startup_parameter(rclcpp::Node & node, const std::string & name, const T & value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node.declare_parameter<T>(name, value, descriptor);
}
template<size_t N>
std::string startup_parameter(rclcpp::Node & node, const std::string & name, const char (&value)[N]) {
    return startup_parameter(node, name, std::string(value));
}

inline bool finite_pose(const lwr_controllers::msg::PoseRPY & pose) {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) && std::isfinite(pose.orientation.roll) &&
           std::isfinite(pose.orientation.pitch) && std::isfinite(pose.orientation.yaw);
}
inline bool finite_twist(const geometry_msgs::msg::Twist & twist) {
    return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
           std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
           std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}
}  // namespace ultrasound_trajectory
