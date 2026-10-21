#include <controller_interface/controller_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "KinematicChainControllerBase.h"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <kdl/chaindynparam.hpp>
#include <memory>
#ifndef BASIC_JOINT_CONTROLLER
#define BASIC_JOINT_CONTROLLER


namespace kuka_controllers{

	class BasicJointController : public controller_interface::KinematicChainControllerBase<lwr_controllers::EffortInterface>
	{
	public:
		BasicJointController();
		~BasicJointController();
		bool configure(lwr_controllers::NodeParameters &n) override;
		void update_command(const rclcpp::Time& time, const rclcpp::Duration& period) override;
		void command(const std_msgs::msg::Float64MultiArray::ConstSharedPtr &msg);
		void starting(const rclcpp::Time& time) override;
		void stopping(const rclcpp::Time& time) override {}
	private:
		rclcpp::SubscriptionBase::SharedPtr sub_posture_;
		KDL::JntArray K_;
		KDL::JntArray D_;
		int cmd_flag_;	// discriminate if a user command arrived
		double lambda;	// flattening coefficient of tanh
		int step_;		// step used in tanh for reaching gradually the desired posture
		KDL::JntArray joint_initial_states_; // joint as measured at the beginning of the control action
		KDL::JntArray current_cmd_; // command value as delta to be added to joint_initial_states_
		KDL::JntArray cmd_states_;
		KDL::JntArray tau_des_;		
		KDL::JntArray tau_cmd_;	


		KDL::JntArray tau_gravity_;	
		std::unique_ptr<KDL::ChainDynParam> id_solver_;// inverse dynamics solver
		KDL::Vector gravity_;	
	

	};
}




#endif
