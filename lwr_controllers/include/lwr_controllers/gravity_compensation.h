#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

namespace lwr_controllers
{
    class GravityCompensation: public controller_interface::KinematicChainControllerBase<lwr_controllers::EffortInterface>
    {
    public:
        
        GravityCompensation();
        ~GravityCompensation();
        
        bool configure(lwr_controllers::NodeParameters &n);
        void starting(const rclcpp::Time& time);
        void update_command(const rclcpp::Time& time, const rclcpp::Duration& period);
        void stopping(const rclcpp::Time& time);
        
    private:
        bool hardware_gravity_compensation_ = true;
        std::unique_ptr<KDL::ChainDynParam> gravity_solver_;
        KDL::JntArray q_, gravity_torques_;
    // private:
        
        // std::vector<float> previous_stiffness_; /// stiffness before activating controller
        
        // hack required as long as there is separate position handle for stiffness
        // std::vector<hardware_interface::JointHandle> joint_stiffness_handles_;
        
        // const static float DEFAULT_STIFFNESS = 0.01;
        
    };
}

#endif