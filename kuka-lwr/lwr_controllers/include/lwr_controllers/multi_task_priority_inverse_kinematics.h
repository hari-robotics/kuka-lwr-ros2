#ifndef LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS__MULTI_TASK_PRIORITY_INVERSE_KINEMATICS_H

#include "PIDKinematicChainControllerBase.h"
#include <lwr_controllers/msg/multi_priority_task.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/scoped_ptr.hpp>
#include <sstream>

namespace lwr_controllers
{
	class MultiTaskPriorityInverseKinematics: public controller_interface::PIDKinematicChainControllerBase<lwr_controllers::EffortInterface>
	{
	public:
		MultiTaskPriorityInverseKinematics();
		~MultiTaskPriorityInverseKinematics();

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

		KDL::Frame x_;		//current pose
		std::vector<KDL::Frame> x_des_;	//desired pose

		KDL::Twist x_err_;

		KDL::JntArray tau_cmd_;

		KDL::Jacobian J_;	//Jacobian
		KDL::Jacobian J_star_; // it will be J_*P_

		Eigen::MatrixXd J_pinv_;//<double,7,6> J_pinv_;

		Eigen::Matrix<double,6,1> e_dot_;
		Eigen::Matrix<double,7,7> I_;
		Eigen::Matrix<double,7,7> P_;

		int msg_id_;
		int cmd_flag_;
		int ntasks_;
		std::vector<bool> on_target_flag_;
		std::vector<int> links_index_;


		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
	};

}

#endif