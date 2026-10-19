#include "lwr_hw/lwr_hw_fri.hpp"
#include <pluginlib/class_list_macros.hpp>
// ros2_control_node replaces the ROS 1 standalone ControllerManager loop.
PLUGINLIB_EXPORT_CLASS(lwr_hw::LWRHWFRI, hardware_interface::SystemInterface)
