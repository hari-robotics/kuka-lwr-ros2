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
#include "geometry_msgs/msg/twist.hpp"
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
									// Force Controller Command
	rclcpp::PublisherBase::SharedPtr pub_force;  // ROS 1: never advertised
	
	lwr_controllers::msg::CartesianImpedancePoint command_msg;
	std_msgs::msg::Bool PID_msg;

	//std_msgs::double command_force_msg; *************************
	
	string frame_id = "0";
	double ee_f_x; double ee_f_y; double ee_f_z;	//	EE FORCE
	double ee_t_x; double ee_t_y; double ee_t_z;	//	EE TORQUE
	double ee_position_x; double ee_position_y; double ee_position_z;	//	EE POSITION (retrieved continuously)
	double roll; double pitch; double yaw;	//	EE PRY ORIENTATION
	double q_x; double q_y; double q_z; double q_w;	//	EE QUAT ORIENTATION
	
	int flag_init = 1; //this flag is to retrieve and save in (xi, yi, zi) the position of the initial configuration of the robot at first step
	enum ControlState {
    IMPEDANCE_CONTROL,
    FORCE_CONTROL
	};
	ControlState state_control = IMPEDANCE_CONTROL; // control initialized to IMPEDANCE CONTROL
	
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
// force threshold
	double threshold_f = -0.1; // SET!! From graphs

	//	SIN Trajectory Variables
	double dx = 0.00005;	//step on X axis
	double a = 0.05;	//0.1	//amplitude of sin wave

	//	QP Variables
//	double K_z = 500;	//	initialize K to default value of cartesian_impedance_controller (?)
	double K_z = 100.0;

	//	PID
	double F_z = 0.0;

	// Force reference

	double df = 0.0005; // step force increment
	double F_z_prev =0.0; 
	bool force_control_executed = false;
	double prev_time=0;
	float check =1.0;
	double new_force_reference = 0;
	std::vector<float> buffer;
	int i_sample=0;
	

public:
	

    rclcpp::Node::SharedPtr node() { return nh; }  // ROS 2: node to spin in main()
	UltrasoundTrajNode(){
		
		sub_ft = nh->create_subscription<lwr_controllers::msg::ArmState>("/lwr/arm_state_controller/arm_state", 10, std::bind(&UltrasoundTrajNode::callback_ft, this, std::placeholders::_1));
		sub_pose = nh->create_subscription<lwr_controllers::msg::PoseRPY>("/cartesian_position/pose/end_effector", 10, std::bind(&UltrasoundTrajNode::callback_pose, this, std::placeholders::_1));
		// sub_stiffness = nh->create_subscription<std_msgs::msg::Float64>("/stiffness", 10, std::bind(&UltrasoundTrajNode::callback_stiffness, this, std::placeholders::_1));
		sub_force_calib = nh->create_subscription<geometry_msgs::msg::Twist>("/Force", 10, std::bind(&UltrasoundTrajNode::callback_force, this, std::placeholders::_1));

		pub_command = nh->create_publisher<lwr_controllers::msg::CartesianImpedancePoint>("/lwr/cartesian_impedance_controller/command", 10);
		pub_PID = nh->create_publisher<std_msgs::msg::Bool>("/enable_PID", 10);
	}

	void callback_ft(const lwr_controllers::msg::ArmState::ConstSharedPtr  ft_msg) {
//		Subscribe to ee force, torque (from arm_state_controller)
// 		Read force from Force Sensor;
		
		ee_f_x = ft_msg->est_ee_wrench_base.force.x;
		ee_f_y = ft_msg->est_ee_wrench_base.force.y;
		ee_f_z = ft_msg->est_ee_wrench_base.force.z;
		ee_t_x = ft_msg->est_ee_wrench_base.torque.x;
		ee_t_y = ft_msg->est_ee_wrench_base.torque.y;
		ee_t_z = ft_msg->est_ee_wrench_base.torque.z;
        
	    
		ee_f_x = ft_msg->est_ee_wrench.force.x;
		ee_f_y = ft_msg->est_ee_wrench.force.y;
		ee_f_z = ft_msg->est_ee_wrench.force.z;
		ee_t_x = ft_msg->est_ee_wrench.torque.x;
		ee_t_y = ft_msg->est_ee_wrench.torque.y;
		ee_t_z = ft_msg->est_ee_wrench.torque.z;
		/*
		ee_f_x = 0.0; ee_f_y = 0.0; ee_f_z = 0.0;
		ee_t_x = 0.0; ee_t_y = 0.0; ee_t_z = 0.0;
		*/
	}

	void callback_force(const geometry_msgs::msg::Twist::ConstSharedPtr  force_msg){
		rclcpp::Time current_time = nh->now(); // Ottiene l'istante temporale corrente
		double current_time1 = current_time.seconds();
		F_z = force_msg -> linear.z-1.66;
		double delta_t = (current_time1 - prev_time);
		//RCLCPP_INFO(nh->get_logger(), "sec: %f",delta_t);	
			//RCLCPP_INFO(nh->get_logger(), "derivata: %f",dF_z_dt);
       		if (F_z < threshold_f && !force_control_executed) {
            	state_control = FORCE_CONTROL;
				RCLCPP_INFO(nh->get_logger(), "State Control = FORCE_CONTROL");
				i_sample++;
				if (i_sample =50) {
						i_sample=0;
						if (buffer.size()>=3){
							buffer.erase(buffer.begin());
						}
						buffer.push_back(F_z);
						if (buffer.size()==3) {
							if (buffer[0]-buffer[1]>0.0002 && buffer[2]-buffer[1]>0.0002){  //  buffer[1]<buffer[0] && buffer[1]<buffer[2]
								state_control = IMPEDANCE_CONTROL; 
								force_control_executed= true; 
								RCLCPP_INFO (nh->get_logger(), "FORCE CONTROL EXECUTED------> IMP CONTROL");
							}
						}		
				}
		RCLCPP_INFO(nh->get_logger(), "BUFFER IS %f, %f, %f", buffer[0], buffer[1], buffer[2]);
        	} 	
			// else if (dF_z_dt > 1.0) {
            // 	state_control = IMPEDANCE_CONTROL; // Cambia lo stato per impedire ulteriori esecuzioni
			// 	RCLCPP_INFO (nh->get_logger(), "FORCE CONTROL EXECUTED");
			// 	force_control_executed= true; // Impedisce future esecuzioni del controllo in forza
        	// }
			
        	prev_time = current_time1;
        	F_z_prev = F_z;
		
		////////////////////////////////////////////////////////////////////////////////////// 
		
		////////////////////////////////////////////////////////////////////////	
		
	}

	void callback_stiffness(const std_msgs::msg::Float64::ConstSharedPtr  stiff_msg){
//		Subscribe to optimized stiffness (from qp_node)
		K_z = stiff_msg->data;
		RCLCPP_INFO(nh->get_logger(), "K_z from qp_node: %f", K_z);
	}


	void applyImpedanceControl() {
	    rclcpp::Rate loop_rate(100);	// 100 Hz
		RCLCPP_INFO (nh->get_logger(), "IMPEDANCE CONTROL");   

		if (i <= T * SubSteps)
		{
			zf = zi - 0.06;
			//zf = zi;
			zt = SmoothTraj(zi, zf, T, SubSteps, i_home);
			cout << zt << endl;

			command_msg.x_fri.position.x = xi;
			command_msg.x_fri.position.y = yi;
			command_msg.x_fri.position.z = zt;
			command_msg.f_fri.force.z = F_z;

			//command_msg.k_fri.z = K_z;	// 500 N/m
			//RCLCPP_INFO(nh->get_logger(), "K_z publishing: %f", K_z);
			pub_command->publish(command_msg);

			i++;
			i_home++;
			//RCLCPP_INFO(nh->get_logger(), "i: %d, x_cmd: %f, y_cmd: %f, z_cmd: %f", i, xi, yi, zt);
			//RCLCPP_INFO(nh->get_logger(), "Moving from the home position to the start position.");
			loop_rate.sleep();
		}

		if (i > T * SubSteps && i <= 2 * T * SubSteps)
		{
			command_msg.x_fri.position.x = xi;
			command_msg.x_fri.position.y = yi;
			command_msg.x_fri.position.z = zt;
			command_msg.f_fri.force.z = F_z;

			// command_msg.k_fri.z = K_z;
			pub_command->publish(command_msg);

			i++;
			//RCLCPP_INFO(nh->get_logger(), "i: %d, x_cmd: %f, y_cmd: %f, z_cmd: %f", i, xi, yi, zt);
			RCLCPP_INFO(nh->get_logger(), "Reached the final position.");
			loop_rate.sleep();
			
		}

		//if (i > 2*T*SubSteps){

		//rclcpp::shutdown();
		
		}
	
	void applyForceControl() {
		rclcpp::Rate loop_rate(100);	
				if (force_control_executed) {
				state_control=IMPEDANCE_CONTROL;
				}
    //Force Control
				if (!force_control_executed){
					double deltaFz = (F_z - F_z_prev);
					double force_increment = 0.1; // Adjust this value as needed
					
					new_force_reference=new_force_reference+force_increment; // Adjusting the z-force directly
					
					// Update the command message with the same position reference
					command_msg.header.frame_id = frame_id;		
					command_msg.x_fri.position.x = ee_position_x;
					command_msg.x_fri.position.y = ee_position_y;
					command_msg.x_fri.position.z = ee_position_z;
			//		Orientation
					command_msg.x_fri.orientation.x = q_x; command_msg.x_fri.orientation.y = q_y; command_msg.x_fri.orientation.z = q_z; command_msg.x_fri.orientation.w = q_w;
			//
					// Update the command message with the new force reference
					command_msg.f_fri.force.x = 0.0;
					command_msg.f_fri.force.y = 0.0;
					if (new_force_reference >= 5){
						new_force_reference=5;
						//RCLCPP_INFO(nh->get_logger(), "FORCE THRESHOLD %f", new_force_reference); 
					}
					
					command_msg.f_fri.force.z = new_force_reference;
					
					
					//Torque		
					command_msg.f_fri.torque.x = 0.0;
					command_msg.f_fri.torque.y = 0.0;
					command_msg.f_fri.torque.z = 0.0;
							
					
					// Publish the updated command


					pub_command->publish(command_msg);
						
					RCLCPP_INFO(nh->get_logger(), "FORCE CONTROL : Force reference: %f", new_force_reference);
					//RCLCPP_INFO(nh->get_logger(), "Position %f",ee_position_z );
					//		}
				}
			
			}
		
	void state_control_function (ControlState &state_control){
		 switch(state_control) {
        case FORCE_CONTROL:
			applyForceControl();
            break;
        case IMPEDANCE_CONTROL:
            applyImpedanceControl();
            break;
        default:
            break;
    }
	}


	void callback_pose(const lwr_controllers::msg::PoseRPY::ConstSharedPtr  pose_msg) {
		
//		Subscribe to current ee position, orientation (from cartesian_position_node)
		//RCLCPP_INFO(nh->get_logger(), "Callback pose");
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
			command_msg.f_fri.force.z = F_z;
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


		    updateControl();
	
	}
	void updateControl() {
    	state_control_function(state_control);
	}


};	//end class

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	
	UltrasoundTrajNode ultrasound_traj_node;
	rclcpp::spin(ultrasound_traj_node.node());
	rclcpp::shutdown();
	return 0;
}
