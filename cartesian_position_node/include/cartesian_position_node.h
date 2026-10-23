#include "sensor_msgs/msg/joint_state.hpp"
#include "lwr_controllers/msg/pose_rpy_stamped.hpp"
#include "lwr_controllers/msg/rpy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
//#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include "forward_kinematics.h"
#include <control_msgs/msg/joint_controller_state.hpp> 
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
//#include <kdl/jointarrayvel.hpp>
#ifndef CARTESIAN_POSITION_NODE
#define CARTESIAN_POSITION_NODE
#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>


void reorder(std::vector<double>& vA, std::vector<size_t>& vI)  
{
size_t i, j, k;
    for(i = 0; i < vA.size(); i++){ 
        while(i != (j = vI[i])){
            k = vI[j];
            std::swap(vA[j], vA[k]);
            std::swap(vI[i], vI[j]);
        }
    }
}


class CartesianPositionNode{


public:
	CartesianPositionNode(rclcpp::Node::SharedPtr n, std::string subname): fk_(), n_(n){
		tt_pub_ = n->create_publisher<lwr_controllers::msg::PoseRPYStamped>("/cartesian_position/pose/tooltip",10);
		ee_pub_ = n->create_publisher<lwr_controllers::msg::PoseRPYStamped>("/cartesian_position/pose/end_effector",10);	
		tt_vel_pub_ = n->create_publisher<geometry_msgs::msg::TwistStamped>("/cartesian_position/vel/tooltip",10);
		ee_vel_pub_ = n->create_publisher<geometry_msgs::msg::TwistStamped>("/cartesian_position/vel/end_effector",10);	
		sub_ = n->create_subscription<sensor_msgs::msg::JointState>(subname, 10, std::bind(&CartesianPositionNode::callback, this, std::placeholders::_1));

		fk_.init(n);


	}

	void callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
	{
		auto received_time = n_->now();
		KDL::JntArray q(msg->position.size());
		KDL::JntArray q_dot(msg->velocity.size());
		KDL::Frame x;
		std::vector<double> x_dot(6,1);
		std::vector<double> js;
		std::vector<double> js_dot;
		std::vector<size_t> order{0,1,6,2,3,4,5};
		// TODO(ros2-migration): ROS 1 joint_state_controller published the joints in
		// alphabetical order (std::map in the hardware_interface resource manager), which the
		// reordering below relies on. The Humble joint_state_broadcaster order is not fixed,
		// so the message is first put back into that order by joint name.
		std::vector<size_t> ros1_order(msg->name.size());
		std::iota(ros1_order.begin(), ros1_order.end(), 0);
		std::sort(ros1_order.begin(), ros1_order.end(), [&msg](size_t a, size_t b) { return msg->name[a] < msg->name[b]; });
		for (int i=0; i<msg->position.size();i++){
			js.push_back(msg->position[ros1_order[i]]);
			js_dot.push_back(msg->velocity[ros1_order[i]]);
		}
		reorder(js,order);
		reorder(js_dot,order);
		for (int i=0; i<js.size();i++){
			q(i) = js[i];
			q_dot(i) = js_dot[i];
		}
		float tool_length;
		
		// tool_length=0.22; //0.275
		tool_length=0.252;

		float d1 = 0.4; 
		float d2 = 0.39; 
		float d3 = tool_length+0.078; 
		float d4 = 0.078;


		fk_.compute(q,x);


		fk_.computeVel(q,q_dot,x_dot,d1,d2,d4); //comment JLF 2022-12-19


		pose_.pose.position.x = x.p.x();
		pose_.pose.position.y = x.p.y();
		pose_.pose.position.z = x.p.z();
		x.M.GetRPY(pose_.pose.orientation.roll,pose_.pose.orientation.pitch, pose_.pose.orientation.yaw);
/* 
		vel_.twist.linear.x = x_dot.at(0);
		vel_.twist.linear.y = x_dot.at(1);
		vel_.twist.linear.z = x_dot.at(2);
		vel_.twist.angular.x = x_dot.at(3);
		vel_.twist.angular.y = x_dot.at(4);
		vel_.twist.angular.z = x_dot.at(5);
*/
			
		// Add timestamps for time alignment (Zihao 2026-10-23)
		pose_.header.stamp = received_time;
		vel_.header.stamp = received_time;

		ee_pub_->publish(pose_);
		ee_vel_pub_->publish(vel_); //comment JLF 2022-12-19

		float tip[3];
		tip[0]=x.M(0,2)*tool_length;
		tip[1]=x.M(1,2)*tool_length;
		tip[2]=x.M(2,2)*tool_length;
		std::cout << tip[0]<< " "<<tip[1]<< " "<<tip[2]<<std::endl;
		x.p[0]=x.p[0]+tip[0];
		x.p[1]=x.p[1]+tip[1];
		x.p[2]=x.p[2]+tip[2];
		std::cout << x.p[0]<< " "<<x.p[1]<< " "<<x.p[2]<<std::endl<<std::endl;
		std::cout << tip[0]<< " "<<tip[1]<< " "<<tip[2]<<std::endl<<std::endl;
		pose_.pose.position.x = x.p.x();
		pose_.pose.position.y = x.p.y();
		pose_.pose.position.z = x.p.z();

		fk_.computeVel(q,q_dot,x_dot,d1,d2,d3);



		vel_.twist.linear.x = x_dot.at(0);
		vel_.twist.linear.y = x_dot.at(1);
		vel_.twist.linear.z = x_dot.at(2);
		vel_.twist.angular.x = x_dot.at(3);
		vel_.twist.angular.y = x_dot.at(4);
		vel_.twist.angular.z = x_dot.at(5);

		double a = 0.7;
		vel_.twist.linear.x = a*vel_.twist.linear.x + (1-a)*vel_previous_.twist.linear.x;
		vel_.twist.linear.y = a*vel_.twist.linear.y + (1-a)*vel_previous_.twist.linear.y;
		vel_.twist.linear.z = a*vel_.twist.linear.z + (1-a)*vel_previous_.twist.linear.z;

		vel_previous_ = vel_;

		RCLCPP_INFO(n_->get_logger(), "Computed forward kinematics: %f %f %f",vel_.twist.linear.x, vel_.twist.linear.y, vel_.twist.linear.z); //comment JLF 2022-12-19

		// Add timestamps for time alignment (Zihao 2026-10-23)
		pose_.header.stamp = received_time;
		vel_.header.stamp = received_time;

		tt_pub_->publish(pose_); 
		tt_vel_pub_->publish(vel_);
	}


private:
	

	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
	rclcpp::Publisher<lwr_controllers::msg::PoseRPYStamped>::SharedPtr tt_pub_, ee_pub_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr tt_vel_pub_, ee_vel_pub_;
	sensor_msgs::msg::JointState js_;
	lwr_controllers::msg::PoseRPYStamped pose_;
	geometry_msgs::msg::TwistStamped vel_;
	geometry_msgs::msg::TwistStamped vel_previous_;
	ForwardKinematics fk_;
	rclcpp::Node::SharedPtr n_;

};




#endif
