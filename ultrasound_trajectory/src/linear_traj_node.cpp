/*
* This file is used for sending a diplacement along z-axis for Luca's experiment
* 2023-09-10
* call pkg from different workspace from catkin_ws: source devel/setup.bash
* //TODO:
	* time synchronizer (msg filters)
	* fill in the command just when they are updated
	* z displacement max 10 cm
*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
//	kuka-lwr Controllers Msgs
#include "lwr_controllers/msg/arm_state.hpp"
#include "lwr_controllers/msg/pose_rpy.hpp"
#include "lwr_controllers/msg/cartesian_impedance_point.hpp"
//	Header Files
#include "../include/ultrasound_trajectory/smooth_traj.h"
#include "../include/ultrasound_trajectory/pseudo_inverse.h"
//	Eigen lib
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>

#include <math.h>
#include <cmath>
#include <string>
#include <vector>

using namespace std;

class UltrasoundTrajNode{

private:
	rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("ultrasound_traj_node");
	rclcpp::SubscriptionBase::SharedPtr sub_pose; 		// Position, Orientation Subscriber
	rclcpp::SubscriptionBase::SharedPtr sub_ft; 		// Force, Torque Subscriber
	rclcpp::SubscriptionBase::SharedPtr sub_stiffness;
	rclcpp::SubscriptionBase::SharedPtr sub_force_calib;
	
	rclcpp::Publisher<lwr_controllers::msg::CartesianImpedancePoint>::SharedPtr pub_command; 	// Impedance Controller Command
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_PID;
	
	lwr_controllers::msg::CartesianImpedancePoint command_msg;
	std_msgs::msg::Bool PID_msg;
	
	string frame_id = "0";
	double ee_f_x; double ee_f_y; double ee_f_z;	//	EE FORCE
	double ee_t_x; double ee_t_y; double ee_t_z;	//	EE TORQUE
	double ee_position_x; double ee_position_y; double ee_position_z;	//	EE POSITION (retrieved continuously)
	double roll; double pitch; double yaw;	//	EE PRY ORIENTATION
	double q_x; double q_y; double q_z; double q_w;	//	EE QUAT ORIENTATION
	
	int flag_init = 1; //this flag is to retrieve and save in (xi, yi, zi) the position of the initial configuration of the robot at first step
	
	//	Predefined Reference Trajectory Variables
	double xi; double yi; double zi;	//	EE initial position (tmp variable that changes during trajectory)
	double x0; double y0; double z0;	//	EE initial position (fixed variable, never modified)
	double x_new; double y_new; double z_new;	//	EE update position (for linear traj on 1 axis)
	double xt; double yt; double zt;
	double xf; double yf; double zf;
//	NOTE: (xt, yt, zt) and (x_new, y_new, z_new) are actually used in the same way
//	double T = 3; //duration of trajectory execution = (T * SubSteps)/loop_rate
//	int SubSteps = 300;	//SubSteps = 1/t of 5th order polynomial interpolation
	int T = 10;
	int SubSteps = 100;
	int i = 0;
	int i_home = 0;
	int i_traj = 0;

	//	PID
	double F_z = 0.0;

public:

    rclcpp::Node::SharedPtr node() { return nh; }  // ROS 2: node to spin in main()
	UltrasoundTrajNode(){
		
		sub_ft = nh->create_subscription<lwr_controllers::msg::ArmState>("/lwr/arm_state_controller/arm_state", 10, std::bind(&UltrasoundTrajNode::callback_ft, this, std::placeholders::_1));
		sub_pose = nh->create_subscription<lwr_controllers::msg::PoseRPY>("/cartesian_position/pose/end_effector", 10, std::bind(&UltrasoundTrajNode::callback_pose, this, std::placeholders::_1));
		sub_force_calib = nh->create_subscription<geometry_msgs::msg::Wrench>("/Force", 10, std::bind(&UltrasoundTrajNode::callback_force, this, std::placeholders::_1));
		pub_command = nh->create_publisher<lwr_controllers::msg::CartesianImpedancePoint>("/lwr/cartesian_impedance_controller/command", 10);
		pub_PID = nh->create_publisher<std_msgs::msg::Bool>("/enable_PID", 10);
	}

	void callback_ft(const lwr_controllers::msg::ArmState::ConstSharedPtr  ft_msg) {
//		Subscribe to ee force, torque (from arm_state_controller)
		/*
		ee_f_x = ft_msg->est_ee_wrench_base.force.x;
		ee_f_y = ft_msg->est_ee_wrench_base.force.y;
		ee_f_z = ft_msg->est_ee_wrench_base.force.z;
		ee_t_x = ft_msg->est_ee_wrench_base.torque.x;
		ee_t_y = ft_msg->est_ee_wrench_base.torque.y;
		ee_t_z = ft_msg->est_ee_wrench_base.torque.z;
        */
	    /*
		ee_f_x = ft_msg->est_ee_wrench.force.x;
		ee_f_y = ft_msg->est_ee_wrench.force.y;
		ee_f_z = ft_msg->est_ee_wrench.force.z;
		ee_t_x = ft_msg->est_ee_wrench.torque.x;
		ee_t_y = ft_msg->est_ee_wrench.torque.y;
		ee_t_z = ft_msg->est_ee_wrench.torque.z;
		*/
		ee_f_x = 0.0; ee_f_y = 0.0; ee_f_z = 0.0;
		ee_t_x = 0.0; ee_t_y = 0.0; ee_t_z = 0.0;
	}

	void callback_force(const geometry_msgs::msg::Wrench::ConstSharedPtr  force_msg){
		RCLCPP_INFO(nh->get_logger(), "STATE ");
		F_z = force_msg -> force.z;
	}

	void callback_pose(const lwr_controllers::msg::PoseRPY::ConstSharedPtr  pose_msg) {	
//		Subscribe to current ee position, orientation (from cartesian_position_node)
		ee_position_x = pose_msg->position.x;
		ee_position_y = pose_msg->position.y;
		ee_position_z = pose_msg->position.z;
		roll = pose_msg->orientation.roll;
		pitch = pose_msg->orientation.pitch;
		yaw = pose_msg->orientation.yaw;
		
//		Convert RPY to quaternion XYZW
		q_x = (sin(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5)) - (cos(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5));
		q_y = (cos(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5)) + (sin(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5));
		q_z = (cos(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5)) - (sin(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5));		
		q_w = (cos(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5)) + (sin(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5));
/*		
		RCLCPP_INFO(nh->get_logger(), "POSITION - x: %f, y: %f, z: %f", ee_position_x, ee_position_y, ee_position_z);
		RCLCPP_INFO(nh->get_logger(), "ORIENTATION - roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
		RCLCPP_INFO(nh->get_logger(), "QUATERNION - q_x: %f, q_y: %f, q_z: %f, q_w: %f", q_x, q_y, q_z, q_w);
*/

////////		From here commands are sent to the REAL ROBOT from cartesian_impedance_controller command_msg

//		Here below update just once (@ 1st iteration) the orientation, stiffness, damping, force and torque command
//		then only the position command will be updated		
		if (flag_init == 1) {
//			Position
			command_msg.header.frame_id = frame_id;		
			command_msg.x_fri.position.x = ee_position_x;
			command_msg.x_fri.position.y = ee_position_y;
			command_msg.x_fri.position.z = ee_position_z;
//			Orientation
			command_msg.x_fri.orientation.x = q_x; command_msg.x_fri.orientation.y = q_y; command_msg.x_fri.orientation.z = q_z; command_msg.x_fri.orientation.w = q_w;
//			Stiffness
			command_msg.k_fri.x = 1500.0; command_msg.k_fri.y = 1500.0; command_msg.k_fri.z = 1500; // 1500 on x, y 200 on z
			command_msg.k_fri.rx = 250.0; command_msg.k_fri.ry = 250.0; command_msg.k_fri.rz = 250.0; // 200
//			Damping (0.7 default)
			command_msg.d_fri.x = 0.8; command_msg.d_fri.y = 0.8; command_msg.d_fri.z = 0.8;
			command_msg.d_fri.rx = 0.8; command_msg.d_fri.ry = 0.8; command_msg.d_fri.rz = 0.8;
//			Force		
			command_msg.f_fri.force.x = 0.0;
			command_msg.f_fri.force.y = 0.0;
			command_msg.f_fri.force.z = 0.0;
//			Torque		
			command_msg.f_fri.torque.x = 0.0;
			command_msg.f_fri.torque.y = 0.0;
			command_msg.f_fri.torque.z = 0.0;
//			Save initial position
			xi = ee_position_x; x0 = ee_position_x;
			yi = ee_position_y; y0 = ee_position_y;
			zi = ee_position_z;
			flag_init = 0;
		}


////////////////		Pi to Pf SMOOTH TRAJECTORY (T=5; SubSteps=100: OK)
//		RPY are kept the same as in initial position
		rclcpp::Rate loop_rate(100);	// 100 Hz
		if (i <= T * SubSteps)
		{
			zf = zi + 0.04;
			//zf = zi;
			zt = SmoothTraj(zi, zf, T, SubSteps, i_home);
			cout << zt << endl;

			command_msg.x_fri.position.x = xi;
			command_msg.x_fri.position.y = yi;
			command_msg.x_fri.position.z = zt;

			// command_msg.f_fri.force.z = 

			//command_msg.k_fri.z = K_z;	// 500 N/m
			//RCLCPP_INFO(nh->get_logger(), "K_z publishing: %f", K_z);
			pub_command->publish(command_msg);

			i++;
			i_home++;
			//RCLCPP_INFO(nh->get_logger(), "i: %d, x_cmd: %f, y_cmd: %f, z_cmd: %f", i, xi, yi, zt);
			//RCLCPP_INFO(nh->get_logger(), "Moving from the home position to the start position.");
			RCLCPP_INFO(nh->get_logger(), "Codice linear_traj_node.cpp");
			
			loop_rate.sleep();
		}

		if (i > T * SubSteps && i <= 2 * T * SubSteps)
		{
			command_msg.x_fri.position.x = xi;
			command_msg.x_fri.position.y = yi;
			command_msg.x_fri.position.z = zt;

			// command_msg.k_fri.z = K_z;
			//pub_command->publish(command_msg);

			i++;
			RCLCPP_INFO(nh->get_logger(), "i: %d, x_cmd: %f, y_cmd: %f, z_cmd: %f", i, xi, yi, zt);
			RCLCPP_INFO(nh->get_logger(), "Reached the final position.");
			loop_rate.sleep();
			
		}

		if (i > 2*T*SubSteps){

		rclcpp::shutdown();
		}
/*
		if (i > 2*T*SubSteps && i <= 4*T*SubSteps){
			yf = yi - 0.08;	//	for final experiments: 8 cm
//			yf = yi - 0.1;	//	for final experiments: 8 cm 	
			yt = SmoothTraj(yi, yf, 10, SubSteps, i_traj);
			
			command_msg.x_fri.position.x = xi;
			command_msg.x_fri.position.y = yt;
//			command_msg.x_fri.position.z = zt;

			command_msg.k_fri.z = K_z;
			pub_command->publish(command_msg);

			i++;
			i_traj++;
//			RCLCPP_INFO(nh->get_logger(), "i: %d, i_traj: %d, x_cmd: %f, y_cmd: %f, z_cmd: %f", i, i_traj, xi, yt, zt);
			RCLCPP_INFO(nh->get_logger(), "Scanning surface...");

//			PID
			PID_msg.data = 1;
			pub_PID->publish(PID_msg);
			loop_rate.sleep();
		}
*/
/*
		if (i > 4*T*SubSteps && i <= 5*T*SubSteps){
			command_msg.x_fri.position.x = xi;
			command_msg.x_fri.position.y = yt;
//			command_msg.x_fri.position.z = zt;

			command_msg.k_fri.z = K_z;
			pub_command->publish(command_msg);

			i++;
//			RCLCPP_INFO(nh->get_logger(), "i: %d, x_cmd: %f, y_cmd: %f, z_cmd: %f", i, xi, yt, zt);
			RCLCPP_INFO(nh->get_logger(), "Reached the end position.");

//			PID
			PID_msg.data = 1;
			pub_PID->publish(PID_msg);
			loop_rate.sleep();
		}
*/
/*
		if (i > 5*T*SubSteps){
			RCLCPP_INFO(nh->get_logger(), "Scanning completed.");
//			command_msg.k_fri.z = K_z;
//			pub_command->publish(command_msg);
			rclcpp::shutdown();
		}

////////////////		SIN TRAJECTORY on X, Y (+ SMOOTHING) (T=10; SubSteps=100; 1/(dx*1000): OK)
/*
		rclcpp::Rate loop_rate(50); 	//10 Hz	
		if (i <= T*SubSteps){
			xf = xi + dx;
			xt = SmoothTraj(xi, xf, T, SubSteps, i);

			yf = y0 + a*sin(2*M_PI*(1/(dx*1000))*(xf-x0));
			yt = SmoothTraj(yi, yf, T, SubSteps, i);

			command_msg.x_fri.position.x = xt;
			command_msg.x_fri.position.y = yt;
			command_msg.x_fri.position.z = zi;
		
			pub_command->publish(command_msg);
			
			xi = xf;
			yi = yf;

  			i++;

  			loop_rate.sleep();
  		}

		if (i > T*SubSteps){
			rclcpp::shutdown();
		}
*/

//	For simulation in GAZEBO: look at the backup file from ~/Ilaria folder
	
//	RCLCPP_INFO(nh->get_logger(), "callback from ultrasound_traj_node");
	}	//end 2nd callback
	
};	//end class

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	
	UltrasoundTrajNode ultrasound_traj_node;
	
	rclcpp::spin(ultrasound_traj_node.node());
	rclcpp::shutdown();
	return 0;
}
