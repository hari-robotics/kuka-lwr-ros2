#ifndef LWR_CONTROLLERS__ONE_TASK_INVERSE_DYNAMICS_JL_H
#define LWR_CONTROLLERS__ONE_TASK_INVERSE_DYNAMICS_JL_H

#include "PIDKinematicChainControllerBase.h"
#include <lwr_controllers/msg/multi_priority_task.hpp>

#include <lwr_controllers/msg/pose_rpy.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>

namespace lwr_controllers
{
	class OneTaskInverseDynamicsJL: public controller_interface::PIDKinematicChainControllerBase<lwr_controllers::EffortInterface>
	{
	public:
		OneTaskInverseDynamicsJL();
		~OneTaskInverseDynamicsJL();

		bool configure(lwr_controllers::NodeParameters &n);
		void starting(const rclcpp::Time& time);
		void update_command(const rclcpp::Time& time, const rclcpp::Duration& period);
		void command(const lwr_controllers::msg::PoseRPY::ConstSharedPtr &msg);
		void set_marker(KDL::Frame x, int id);
		double task_objective_function(KDL::JntArray q);

	private:
		rclcpp::SubscriptionBase::SharedPtr sub_command_;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_error_;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pose_;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

		std_msgs::msg::Float64MultiArray msg_err_;
		std_msgs::msg::Float64MultiArray msg_pose_;
		visualization_msgs::msg::Marker msg_marker_;
		std::stringstream sstr_;
        
		KDL::JntArray qdot_last_;

		KDL::Frame x_,x0_;	//current e-e pose
		Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity

		KDL::Frame x_des_;	//desired pose
		KDL::Twist x_des_dot_;
		KDL::Twist x_des_dotdot_;

		KDL::Twist x_err_;

		KDL::JntArray Kp_,Kd_;

		KDL::JntArray tau_;

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

		KDL::Jacobian J_;	//Jacobian J(q)
		KDL::Jacobian J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;	//d/dt(J(q))
		KDL::Jacobian J_star_; // it will be J_*P_

		Eigen::MatrixXd J_pinv_;

		Eigen::Matrix<double,6,1> e_ref_;
		Eigen::Matrix<double,7,7> I_;
		Eigen::Matrix<double,7,7> N_trans_;
		Eigen::MatrixXd M_inv_;
		Eigen::MatrixXd omega_;
		Eigen::MatrixXd lambda_;
		Eigen::Matrix<double,6,1> b_;

		double phi_;	
		double phi_last_;

		int step_;
		int first_step_;
		int msg_id_;
		int cmd_flag_;
		int ntasks_;
		bool on_target_flag_;
		int links_index_;


		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
		//boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
		//boost::scoped_ptr<KDL::ChainFkSolverAcc_recursive> fk_acc_solver_;
	};

}

#endif