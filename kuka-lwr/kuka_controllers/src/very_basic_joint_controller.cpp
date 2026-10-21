#include <controller_interface/controller_interface.hpp>
#include <lwr_controllers/KinematicChainControllerBase.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/float64.hpp>



namespace kuka_controllers{



class VeryBasicJointController : public lwr_controllers::ControllerAdapter<lwr_controllers::EffortInterface>
{


bool configure(lwr_controllers::NodeParameters& n) override
{
	nh_ = n;
	std::string the_joint;
	if (!n.getParam("joint", the_joint))
	{
		RCLCPP_ERROR(n.node()->get_logger(), "Gibb joint name");
		return false;
	}
	joint_names_ = {the_joint};
	joint_handles_ = {lwr_controllers::JointHandle{the_joint}}; // get joint handle
	// TODO(ros2-migration): ROS 1 read command_ = joint_.getPosition() here (controller load);
	// the state interface is loaned only at activation, so the read is done at the first
	// activation after configure (starting()) instead, see MIGRATION_NOTES.
	command_read_ = false;
	if (!n.getParam("gain", gain_))
	{
		RCLCPP_ERROR(n.node()->get_logger(), "Gibb gain value");
		return false;
	}

	// Sub
	sub_command_ = n.subscribe("command", 1, &VeryBasicJointController::command, this);
	return true;

}

void update_command(const rclcpp::Time& time, const rclcpp::Duration& period) override
{
	double err = command_ - joint_handles_[0].getPosition();
	double effort = err * gain_;
	joint_handles_[0].setCommand(effort);

}

void command(const std_msgs::msg::Float64::ConstSharedPtr& msg)
{
	command_ = msg->data;

}

void starting(const rclcpp::Time& time) override
{
	if (!command_read_) { command_ = joint_handles_[0].getPosition(); command_read_ = true; } // current joint goal
}
void stopping(const rclcpp::Time& time) override { }


private:
	double gain_;
	double command_;
	bool command_read_ = false;
	rclcpp::SubscriptionBase::SharedPtr sub_command_;


};


}

PLUGINLIB_EXPORT_CLASS(kuka_controllers::VeryBasicJointController, controller_interface::ControllerInterface);
