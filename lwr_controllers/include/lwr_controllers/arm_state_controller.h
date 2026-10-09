#ifndef ARM_STATE_CONTROLLER_H
#define ARM_STATE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <lwr_controllers/msg/arm_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <realtime_tools/realtime_publisher.h>

#include <boost/scoped_ptr.hpp>

namespace arm_state_controller
{
    class ArmStateController: public controller_interface::KinematicChainControllerBase<lwr_controllers::StateInterface>
    {
    public:
        
        ArmStateController();
        ~ArmStateController();
        
        bool configure(lwr_controllers::NodeParameters &n);
        void starting(const rclcpp::Time& time);
        void update_command(const rclcpp::Time& time, const rclcpp::Duration& period);
        void stopping(const rclcpp::Time& time);
        
    private:
                
        std::shared_ptr< realtime_tools::RealtimePublisher< lwr_controllers::msg::ArmState > > realtime_pub_;
        
        boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_;
        boost::scoped_ptr<KDL::Jacobian> jacobian_;
        boost::scoped_ptr<KDL::Vector> gravity_;
        boost::scoped_ptr<KDL::JntArray> joint_position_;
        boost::scoped_ptr<KDL::JntArray> joint_velocity_;
        boost::scoped_ptr<KDL::JntArray> joint_acceleration_;
        boost::scoped_ptr<KDL::Wrenches> joint_wrenches_;
        boost::scoped_ptr<KDL::JntArray> joint_effort_est_;
        
        rclcpp::Time last_publish_time_;
        double publish_rate_;
    };
}

#endif