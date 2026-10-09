
#ifndef LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <boost/scoped_ptr.hpp>

/*
	tau_cmd_ = K_*(q_des_ - q_msr_) + D_*dotq_msr_ + G(q_msr_)

*/

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
        
	private:
		std::vector<JointHandle> joint_stiffness_handles_, joint_damping_handles_, joint_set_point_handles_;

		rclcpp::SubscriptionBase::SharedPtr sub_stiffness_, sub_damping_, sub_add_torque_;
		rclcpp::SubscriptionBase::SharedPtr sub_posture_;

		KDL::JntArray q_des_;
		KDL::JntArray tau_des_;
		KDL::JntArray K_, D_;

	};

} // namespace

#endif
