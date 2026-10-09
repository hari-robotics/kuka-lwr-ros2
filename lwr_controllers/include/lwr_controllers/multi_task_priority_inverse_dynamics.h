#ifndef LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_DYNAMICS_H
#define LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_DYNAMICS_H

#include "PIDKinematicChainControllerBase.h"
#include <lwr_controllers/msg/multi_priority_task.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/scoped_ptr.hpp>
#include <sstream>

namespace lwr_controllers
{
	class MultiTaskPriorityInverseDynamics: public controller_interface::PIDKinematicChainControllerBase<lwr_controllers::EffortInterface>
	{
	public:
		MultiTaskPriorityInverseDynamics();
		~MultiTaskPriorityInverseDynamics();

		bool configure(lwr_controllers::NodeParameters &n);
		void starting(const rclcpp::Time& time);
		void update_command(const rclcpp::Time& time, const rclcpp::Duration& period);
		void command(const lwr_controllers::msg::MultiPriorityTask::ConstSharedPtr &msg);
		void set_marker(KDL::Frame x, int index, int id);

	private:
		rclcpp::SubscriptionBase::SharedPtr sub_command_;
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_error_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;

		std_msgs::msg::Float64MultiArray msg_err_;
		visualization_msgs::msg::MarkerArray msg_marker_;
		std::stringstream sstr_;

		KDL::JntArray qdot_last_;

		KDL::Frame x_;	//current e-e pose
		Eigen::Matrix<double,6,1> x_dot_;	//current e-e velocity
		KDL::Twist x_dot_dot_;	//current e-e acceleration
		std::vector<KDL::Frame> x_des_;	//desired pose

		KDL::Twist x_err_;	// position error

		KDL::JntArray Kp_,Kd_;	// velocity error, position error

		KDL::JntArray tau_;	// control torque

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

		KDL::Jacobian J_;	//Jacobian J(q)
		std::vector<KDL::Jacobian> J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;	//d/dt(J(q))

		Eigen::MatrixXd J_pinv_;	// Jacobian pseudo-inv

		Eigen::Matrix<double,6,1> e_ref_;	// reference error
		Eigen::Matrix<double,7,7> I_;		// Identity
		Eigen::Matrix<double,7,7> N_trans_;	// null-space matrix
		Eigen::MatrixXd M_inv_;
		Eigen::MatrixXd omega_;
		Eigen::MatrixXd lambda_;
		Eigen::Matrix<double,6,1> b_;

		int first_step_;	// first step flag
		int msg_id_;		// marker message id
		int cmd_flag_;		// command received flag
		int ntasks_;		// # of task
		std::vector<bool> on_target_flag_;	
		std::vector<int> links_index_;


		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
	};

}

#endif