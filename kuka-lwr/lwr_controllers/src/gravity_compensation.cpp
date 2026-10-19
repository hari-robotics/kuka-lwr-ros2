#include <pluginlib/class_list_macros.hpp>
#include <math.h>

#include <lwr_controllers/gravity_compensation.h>

namespace lwr_controllers 
{
    GravityCompensation::GravityCompensation() {}
    GravityCompensation::~GravityCompensation() {}
    
    bool GravityCompensation::configure(lwr_controllers::NodeParameters &n)
    {
        if (!KinematicChainControllerBase<lwr_controllers::EffortInterface>::configure(n)) return false;
        
        //find stiffness (dummy) joints; this is necessary until proper position/stiffness/damping interface exists
        //for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
        //{
        //     joint_stiffness_handles_.push_back(robot->getHandle(it->getJoint().getName()+"_stiffness"));
        //}
        
        //RCLCPP_DEBUG(nh_.node()->get_logger(), "found %lu stiffness handles", joint_stiffness_handles_.size());
        
        //previous_stiffness_.resize(joint_stiffness_handles_.size());
        
        return true;
    }
    
    void GravityCompensation::starting(const rclcpp::Time& time)
    {
       // for(size_t i=0; i<joint_handles_.size(); i++)
       // {
       //     previous_stiffness_[i] = joint_stiffness_handles_[i].getPosition();
       // }
    }
    
    void GravityCompensation::update_command(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // update the commanded position to the actual, so that the robot doesn't 
        // go back at full speed to the last commanded position when the stiffness 
        // is raised again
        for(size_t i=0; i<joint_handles_.size(); i++) 
        {
            //joint_handles_[i].setCommand(joint_handles_[i].getPosition());
            joint_handles_[i].setCommand( 0.1 );
            //joint_stiffness_handles_[i].setCommand(0.01);
        }
    }
    
    void GravityCompensation::stopping(const rclcpp::Time& time)
    {
        //for(size_t i=0; i<joint_handles_.size(); i++)
        //{
        //    joint_stiffness_handles_[i].setCommand(previous_stiffness_[i]);
        //}
    }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::GravityCompensation, controller_interface::ControllerInterface)
