#include "rclcpp/rclcpp.hpp"
#include "../include/forward_kinematics.h"
#include "../include/cartesian_position_node.h"



int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	// ros::init() removed the ROS arguments from argv; argv[1] is the joint state topic
	std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
	auto n = std::make_shared<rclcpp::Node>("cartesian_position_node");
	CartesianPositionNode CP(n, args.at(1));
	rclcpp::spin(n);
	rclcpp::shutdown();
	return 0;





}
