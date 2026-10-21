#include <pluginlib/class_list_macros.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "../include/basic_joint_controller.h"


namespace kuka_controllers
{

	BasicJointController::BasicJointController(){}
	BasicJointController::~BasicJointController(){}

	bool BasicJointController::configure(lwr_controllers::NodeParameters &n)
	
	{
		KinematicChainControllerBase<lwr_controllers::EffortInterface>::configure(n);
	
		K_.resize(kdl_chain_.getNrOfJoints());
		D_.resize(kdl_chain_.getNrOfJoints());
		cmd_states_.resize(kdl_chain_.getNrOfJoints());
		current_cmd_.resize(kdl_chain_.getNrOfJoints());
		sub_posture_ = nh_.subscribe("command", 1, &BasicJointController::command, this);
		tau_des_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		tau_gravity_.resize(kdl_chain_.getNrOfJoints());
		// TODO(ros2-migration): ROS 1 read the joint handles here (controller load). The
		// ros2_control state interfaces are loaned only at activation, so these reads return 0;
		// joint_msr_states_/joint_des_states_ are not used by the control law below.
		for(int i=0; i<joint_handles_.size(); i++)
		{	

			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
	
			joint_des_states_.q(i) = joint_msr_states_.q(i);
			joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);

			tau_des_(i) = 0;
			if ( !nh_.getParam("stiffness_gains", K_(i) ) ){
				RCLCPP_WARN(nh_.node()->get_logger(), "Stiffness gain not set in yaml file, Using %f", K_(i));
			}
			if ( !nh_.getParam("damping_gains", D_(i)) ){
				RCLCPP_WARN(nh_.node()->get_logger(), "Damping gain not set in yaml file, Using %f", D_(i));
			}
		}
		gravity_ = KDL::Vector::Zero();
		gravity_(2) = -9.81;
		id_solver_.reset( new KDL::ChainDynParam(kdl_chain_, gravity_) );

		return true;
	}

	void BasicJointController::starting(const rclcpp::Time& time)
	{

	}


	void BasicJointController::update_command(const rclcpp::Time& time, const rclcpp::Duration& period)
	{
	
		for (size_t i=0; i<joint_handles_.size(); i++)
		{
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
		}
		
		// control law
		id_solver_->JntToGravity(joint_msr_states_.q, tau_gravity_);
	        for(size_t i=0; i<joint_handles_.size(); i++)
		
		{
		
		    tau_cmd_(i) = K_(i)*(cmd_states_(i) - joint_msr_states_.q(i)) + D_(i)*(joint_msr_states_.qdot(i)) + tau_des_(i) + tau_gravity_(i);
                    
		    joint_handles_[i].setCommand(tau_cmd_(i));
		}
		
	}


void BasicJointController::command(const std_msgs::msg::Float64MultiArray::ConstSharedPtr &msg)
    {
    	if(msg->data.size() == 0)
    		RCLCPP_INFO(nh_.node()->get_logger(), "Desired configuration must be of dimension %lu", joint_handles_.size());
    	else if(msg->data.size() != joint_handles_.size())
    	{
    		RCLCPP_ERROR(nh_.node()->get_logger(), "Posture message had the wrong size: %u", (unsigned int)msg->data.size());
    		return;
    	}
    	else
    	{

    		for (unsigned int i = 0; i<joint_handles_.size(); i++)
    			cmd_states_(i) = msg->data[i];


    	}

	}

}

PLUGINLIB_EXPORT_CLASS(kuka_controllers::BasicJointController, controller_interface::ControllerInterface)






