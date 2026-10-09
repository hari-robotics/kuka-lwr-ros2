#include <rclcpp/rclcpp.hpp>
#include <lwr_controllers/msg/pose_rpy.hpp>
#include <kdl/tree.hpp>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>


#define PI 3.141592653

rclcpp::Subscription<lwr_controllers::msg::PoseRPY>::SharedPtr sub_terminal;

rclcpp::Publisher<lwr_controllers::msg::PoseRPY>::SharedPtr pub_right;
rclcpp::Publisher<lwr_controllers::msg::PoseRPY>::SharedPtr pub_left;

Eigen::Matrix<double,3,1> p_global_r, p_global_l, p_right, p_left;

geometry_msgs::msg::TransformStamped transform_right;
geometry_msgs::msg::TransformStamped transform_left;

double L = 0.1;

Eigen::Matrix<double,3,3> rot_right, rot_left;
Eigen::Matrix<double,3,3> rot_des_r, rot_des_l;
Eigen::Matrix<double,3,3> rot_fin_r, rot_fin_l;
Eigen::Matrix<double,3,1> pos_right, pos_left;

Eigen::Matrix<double,3,3> quat_rot(double x, double y, double z, double w) {
	Eigen::Matrix<double,3,3> temp;
	temp << 1-2*pow(y,2)-2*pow(z,2),             2*x*y-2*w*z,             2*x*z+2*w*y,
			            2*x*y+2*w*z, 1-2*pow(x,2)-2*pow(z,2),             2*z*y-2*w*x,
			            2*x*z-2*w*y,             2*z*y+2*w*x, 1-2*pow(x,2)-2*pow(y,2);
	return temp;
}

int sign(double n) {
	if (n > 0)
		return 1;
	else if (n < 0)
		return -1;
	else 
		return 0;
}

Eigen::Matrix<double,4,1> rot_quat(Eigen::Matrix<double,3,3> r) {
	Eigen::Matrix<double,4,1> temp;
	temp << 0.5*sign(r(2,1)-r(1,2))*sqrt(r(0,0)-r(1,1)-r(2,2)+1),
			0.5*sign(r(0,2)-r(2,0))*sqrt(r(1,1)-r(2,2)-r(0,0)+1),
			0.5*sign(r(1,0)-r(0,1))*sqrt(r(2,2)-r(0,0)-r(1,1)+1),
			0.5*sqrt(r(0,0)+r(1,1)+r(2,2)+1);
	return temp;
}

void gposeCallback(const lwr_controllers::msg::PoseRPY::ConstSharedPtr& msg) {
	KDL::Rotation rot_msg = KDL::Rotation::EulerZYX(msg->orientation.yaw, msg->orientation.pitch, msg->orientation.roll);
	double t1,t2,t3,t4;
	rot_msg.GetQuaternion(t1,t2,t3,t4);
	Eigen::Matrix<double,3,3> rot_msg_mat;
	rot_msg_mat = quat_rot(t1,t2,t3,t4);
	
	Eigen::Matrix<double,3,1> vec_r, vec_l;
	vec_r << 0, L/2, 0;
	vec_l << 0, -L/2, 0;
	p_global_r = rot_msg_mat*vec_r;
	p_global_l = rot_msg_mat*vec_l;
	p_global_r << msg->position.x + p_global_r(0), msg->position.y + p_global_r(1), msg->position.z + p_global_r(2);
	p_global_l << msg->position.x + p_global_l(0), msg->position.y + p_global_l(1), msg->position.z + p_global_l(2);
	
	p_right = rot_right*p_global_r + pos_right;
	p_left = rot_left*p_global_l + pos_left;
	
	KDL::Rotation rot_glob = KDL::Rotation::EulerZYX(msg->orientation.yaw,msg->orientation.pitch,msg->orientation.roll);
	KDL::Rotation rot_des_r_kdl = rot_glob;
	rot_des_r_kdl.DoRotX(PI/2);
	KDL::Rotation rot_des_l_kdl = rot_glob;
	rot_des_l_kdl.DoRotX(-PI/2);
	
	Eigen::Matrix<double,3,3> rot_des_r, rot_des_l;
	
	rot_des_r_kdl.GetQuaternion(t1,t2,t3,t4);
	rot_des_r = quat_rot(t1,t2,t3,t4);
	rot_des_l_kdl.GetQuaternion(t1,t2,t3,t4);
	rot_des_l = quat_rot(t1,t2,t3,t4);
	
	rot_fin_r = rot_right*rot_des_r;
	rot_fin_l = rot_left*rot_des_l;
	
	Eigen::Matrix<double,4,1> r, l;
	r = rot_quat(rot_fin_r);
	l = rot_quat(rot_fin_l);
	KDL::Rotation q_r = KDL::Rotation::Quaternion(r(0),r(1),r(2),r(3));
	KDL::Rotation q_l = KDL::Rotation::Quaternion(l(0),l(1),l(2),l(3));
	
	lwr_controllers::msg::PoseRPY msg_right;
	lwr_controllers::msg::PoseRPY msg_left;
	
	q_r.GetEulerZYX(msg_right.orientation.yaw, msg_right.orientation.pitch, msg_right.orientation.roll);
	q_l.GetEulerZYX(msg_left.orientation.yaw, msg_left.orientation.pitch, msg_left.orientation.roll);
	
	msg_left.id = 0;
	msg_left.position.x = p_left(0);
	msg_left.position.y = p_left(1);
	msg_left.position.z = p_left(2);
	
	msg_right.id = 0;
	msg_right.position.x = p_right(0);
	msg_right.position.y = p_right(1);
	msg_right.position.z = p_right(2);
	
	pub_right->publish(msg_right);
	pub_left->publish(msg_left);
}

int main(int argc, char **argv) {
	// Initialize the node
    rclcpp::init(argc, argv);
    auto node=rclcpp::Node::make_shared("command_vito");
    tf2_ros::Buffer buffer(node->get_clock());
    tf2_ros::TransformListener listener(buffer);
    try {
        transform_right=buffer.lookupTransform("right_arm_base_link","world",tf2::TimePointZero,tf2::durationFromSec(5.0));
        transform_left=buffer.lookupTransform("left_arm_base_link","world",tf2::TimePointZero,tf2::durationFromSec(5.0));
    } catch(const tf2::TransformException &error) {
        RCLCPP_ERROR(node->get_logger(),"%s",error.what());rclcpp::shutdown();return 1;
    }
    auto right=tf2::transformToEigen(transform_right);
    auto left=tf2::transformToEigen(transform_left);
    rot_right=right.rotation();rot_left=left.rotation();
    pos_right=right.translation();pos_left=left.translation();
    sub_terminal=node->create_subscription<lwr_controllers::msg::PoseRPY>("/global_pose",30,gposeCallback);
    pub_right=node->create_publisher<lwr_controllers::msg::PoseRPY>("/right_arm/joint_impedance_controller/command",30);
    pub_left=node->create_publisher<lwr_controllers::msg::PoseRPY>("/left_arm/joint_impedance_controller/command",30);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
