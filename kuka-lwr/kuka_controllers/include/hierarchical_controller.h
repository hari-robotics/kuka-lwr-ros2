#ifndef HIERARCHICAL_CONTROLLER
#define HIERARCHICAL_CONTROLLER

#include "KinematicChainControllerBase.h"
#include <lwr_controllers/msg/pose_rpy.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <memory>
#include <sstream>
#include <fstream>

extern int des_flag;

namespace kuka_controllers
{
	class HierarchicalController: public controller_interface::KinematicChainControllerBase<lwr_controllers::PositionInterface>
	{
	public:
		HierarchicalController();
		~HierarchicalController();

		bool configure(lwr_controllers::NodeParameters &n) override;
		void starting(const rclcpp::Time& time) override;
		void update_command(const rclcpp::Time& time, const rclcpp::Duration& period) override;
		void command(const lwr_controllers::msg::PoseRPY::ConstSharedPtr &msg);

	private:
		rclcpp::SubscriptionBase::SharedPtr sub_command_;
		rclcpp::SubscriptionBase::SharedPtr sub_gains_;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_rcm_;

		KDL::Frame x_;		//current pose
		KDL::Frame x_des_;	//desired pose
		KDL::Frame x_rcm_;      //current rcm pose
		KDL::Frame x_des_rcm_;  //desired rcm pose
		KDL::Frame x_last_;	//last pose
		KDL::Vector pose_rcm_;	//position of the RCM
		KDL::Vector toolvector_;//vector of the tool direction

		float d1,d2,d3,d4;
		float Kp1, Kp2, Kd1, Kd2;
		KDL::Twist x_rcm_err_, x_rcm_err_last_, x_rcm_err_vel_;
		KDL::Twist x_err_, x_err_last_, x_err_vel_;
		KDL::JntArray q_cmd_; // computed set points

		KDL::Jacobian J_end_;	//Jacobian
		KDL::Jacobian J_tooltip_;  //Jacobian of the wrist
		
		Eigen::MatrixXd J_end_pinv_;
		Eigen::MatrixXd J_tooltip_pinv_;
		Eigen::MatrixXd N_;
		Eigen::Matrix<double,3,3> skew_;

		float alpha_;
                
                KDL::Vector goal_orientation;
                KDL::Vector end_orientation;
		KDL::JntArrayAcc joint_des_states_last_;
		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_curr_, quat_des_;

		KDL::Vector v_temp_;
		
		int cmd_flag_;
		// ROS 2: the measured-state initialisation of ROS 1 init() runs at the first activation
		bool init_state_read_ = false;
		
		std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
		std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
		std::ofstream errorfile;
		std::ofstream xfile;
		std::ofstream rcmfile;
	};

}

#endif
