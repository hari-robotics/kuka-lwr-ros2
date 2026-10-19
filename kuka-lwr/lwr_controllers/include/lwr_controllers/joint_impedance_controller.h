#ifndef LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <boost/scoped_ptr.hpp>

//	ADDITIONS 11.22
#include <Eigen/Dense>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
//	ADDITIONS 11.22 - end

/*
	tau_cmd_ = K_*(q_des_ - q_msr_) + D_*dotq_msr_ + G(q_msr_)

*/
using namespace Eigen;

namespace lwr_controllers
{

	class JointImpedanceController: public controller_interface::KinematicChainControllerBase<lwr_controllers::EffortInterface>
	{
	public:

		JointImpedanceController();
		~JointImpedanceController();

		bool configure(lwr_controllers::NodeParameters &n);

		controller_interface::InterfaceConfiguration command_interface_configuration() const override;
		void starting(const rclcpp::Time& time);
		void update_command(const rclcpp::Time& time, const rclcpp::Duration& period);
		void command(const std_msgs::msg::Float64MultiArray::ConstSharedPtr &msg);
		void setParam(const std_msgs::msg::Float64MultiArray::ConstSharedPtr &msg, KDL::JntArray* array, std::string s);
		
        void updateStiffness(const std_msgs::msg::Float64::ConstSharedPtr &stiff_des_msg);	//	added 12.12.22
        
	private:
		// ROS 1: handles of KinematicChainControllerBase<EffortJointInterface>
		std::vector<JointHandle> joint_stiffness_handles_, joint_damping_handles_, joint_set_point_handles_;

		rclcpp::SubscriptionBase::SharedPtr sub_stiffness_, sub_damping_, sub_add_torque_;
		rclcpp::SubscriptionBase::SharedPtr sub_posture_;
		
		rclcpp::SubscriptionBase::SharedPtr sub_est_ext_trq_;	//	Subscriber to /est_ext_torque from lwr_hw.cpp
		rclcpp::SubscriptionBase::SharedPtr sub_stiff;			//	Subscriber to /stiffness from qp_node	
		
		//	Publish EE linear velocity - DEBUG
		rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_vel;
		geometry_msgs::msg::Point vel_msg;
		
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_jac;
		geometry_msgs::msg::Twist vel_msg_jac;
		
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cart_force;
		geometry_msgs::msg::Twist cart_force_msg;

		KDL::JntArray q_des_;
		KDL::JntArray tau_des_;
		KDL::JntArray K_, D_;
		
//	ADDITIONS 10.22		
		KDL::JntArray tau_;
		KDL::JntArray tau_N_;		//	Null space additional torque
		KDL::JntArray tau_d_;		//	desired tau = tau_task + tau_null_space
		KDL::JntArray est_ext_trq_;	//	Estimated external torques

		KDL::Jacobian J_;			//	Jacobian
		KDL::JntSpaceInertiaMatrix M_; //	Inertia Matrix
		
		KDL::JntArray q_start_;	
		bool q_start_initialized_ = false;	// ROS 2: q_start_ is read at the first activation
		KDL::JntArray G_;			//	Gravity
		KDL::JntArray C_;			//	Coriolis
				
//	Eigen matrices for null space projector computation
		Eigen::MatrixXd N_ =  Eigen::MatrixXd(7,7);
		Eigen::MatrixXd J_Eigen_ =  Eigen::MatrixXd(6,7);
		Eigen::MatrixXd M_Eigen_ =  Eigen::MatrixXd(7,7);
		MatrixXd I_ = MatrixXd::Identity(7, 7);
			
//	Conversion from cartesian to joint space
		KDL::JntArray CartesianForces;
		KDL::JntArray CartesianForces_orientation; //TODO: delete
		KDL::Frame x_des_;  	 //desired cart pose
        KDL::Frame x_meas_;  	 //measured cart pose (from direct kinematics)
        KDL::Frame x_meas_old_;  //measured cart pose 
        	
//	Conversion from end-effector to tooltip        	
        float tool_length = 0.252;
        float tip_x_des_[3];
        float tip_x_meas_[3];
        float tip_x_meas_old_[3];
		KDL::Frame x_des_tt_;
		KDL::Frame x_meas_tt_;
		KDL::Frame x_meas_old_tt_;
		
//	Orientation constraint
		KDL::JntArray vel_linear_;		
		
		//	desired orientation
		double qx_des_;
		double qy_des_;
		double qz_des_;
		double qw_des_;
		Eigen::Quaterniond orientation_des_{Eigen::Quaterniond::Identity()};
		
		//	measured orientation
		double qx_;
		double qy_;
		double qz_;
		double qw_;
		Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};
						
		Eigen::Matrix<double, 6, 1> vel_ee_;
		Eigen::Matrix<double, 6, 1> vel_ee_old_;
		Eigen::Matrix<double, 7, 1> vel_jnt_;
		
		Eigen::Matrix<double,7,1> q0_;	//TODO: delete

//	Cartesian and joint Stiffness and Damping
		KDL::JntArray K_joint, D_joint;
		KDL::JntArray K_cart, D_cart; 
		       
//	Solvers
		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
       	boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
       	boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

//	ADDITIONS 10.22 - end		

	};

} // namespace

#endif
