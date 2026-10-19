#ifndef LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H
#define LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/msg/float64_multi_array.hpp>

#include <boost/scoped_ptr.hpp>

namespace lwr_controllers
{
	class ComputedTorqueController: public controller_interface::KinematicChainControllerBase<lwr_controllers::EffortInterface>
	{
	public:

		ComputedTorqueController();
		~ComputedTorqueController();

		bool configure(lwr_controllers::NodeParameters &n);
		void starting(const rclcpp::Time& time);
		void update_command(const rclcpp::Time& time, const rclcpp::Duration& period);
		void command(const std_msgs::msg::Float64MultiArray::ConstSharedPtr &msg);
		void set_gains(const std_msgs::msg::Float64MultiArray::ConstSharedPtr &msg);

	private:

		rclcpp::SubscriptionBase::SharedPtr sub_posture_;
		rclcpp::SubscriptionBase::SharedPtr sub_gains_;
        
		KDL::JntArray cmd_states_;
		int cmd_flag_;	// discriminate if a user command arrived
		double lambda;	// flattening coefficient of tanh
		int step_;		// step used in tanh for reaching gradually the desired posture
		KDL::JntArray joint_initial_states_; // joint as measured at the beginning of the control action
		KDL::JntArray current_cmd_; // command value as delta to be added to joint_initial_states_

		KDL::JntArray tau_cmd_;
		KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
		KDL::JntArray C_, G_;	//Coriolis and Gravitational matrices
		KDL::JntArray Kp_, Kv_;	//Position and Velocity gains

		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

	};
}

#endif