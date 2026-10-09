#include <pluginlib/class_list_macros.hpp>
#include <math.h>

#include <lwr_controllers/arm_state_controller.h>
#include <lwr_controllers/ros2_adapter.h>
#include <utils/pseudo_inversion.h>
#include <control_toolbox/filters.hpp>

namespace arm_state_controller  
{
    ArmStateController::ArmStateController() {}
    ArmStateController::~ArmStateController() {}
    
    bool ArmStateController::configure(lwr_controllers::NodeParameters &n)
    {
        if (!KinematicChainControllerBase<lwr_controllers::StateInterface>::configure(n)) return false;
        
        // get publishing period
        if (!nh_.getParam("publish_rate", publish_rate_)) {
            RCLCPP_ERROR(nh_.node()->get_logger(), "Parameter 'publish_rate' not set");
            return false;
        }
        
        realtime_pub_.reset(new realtime_tools::RealtimePublisher<lwr_controllers::msg::ArmState>(nh_.advertise<lwr_controllers::msg::ArmState>("arm_state", 4)));
        realtime_pub_->msg_.est_ext_torques.resize(kdl_chain_.getNrOfJoints());
        
        gravity_.reset(new KDL::Vector(0.0, 0.0, -9.81)); // TODO: compute from actual robot position (TF?)
        id_solver_.reset(new KDL::ChainIdSolver_RNE(kdl_chain_, *gravity_));
        jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        jacobian_.reset(new KDL::Jacobian(kdl_chain_.getNrOfJoints()));
        joint_position_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        joint_velocity_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        joint_acceleration_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        joint_wrenches_.reset(new KDL::Wrenches(kdl_chain_.getNrOfJoints()));
        joint_effort_est_.reset(new KDL::JntArray(kdl_chain_.getNrOfJoints()));
        
        return true;		
    }
    
    void ArmStateController::starting(const rclcpp::Time& time)
    {
        last_publish_time_ = time;
        for (unsigned i = 0; i < joint_handles_.size(); i++){
            (*joint_position_)(i) = joint_handles_[i].getPosition();
            (*joint_velocity_)(i) = joint_handles_[i].getVelocity();
            (*joint_acceleration_)(i) = 0; 
        }
        RCLCPP_INFO(nh_.node()->get_logger(), "started");
    }
    
    void ArmStateController::update_command(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // limit rate of publishing
        if (publish_rate_ > 0.0 && last_publish_time_ + rclcpp::Duration::from_seconds(1.0/publish_rate_) < time){
            
            if (realtime_pub_->trylock()) {
                // we're actually publishing, so increment time
                last_publish_time_ = last_publish_time_ + rclcpp::Duration::from_seconds(1.0/publish_rate_);
                
                for (unsigned i = 0; i < joint_handles_.size(); i++) {
                    float acceleration = filters::exponentialSmoothing((joint_handles_[i].getVelocity() - (*joint_velocity_)(i))/period.seconds(), (*joint_acceleration_)(i), 0.2);
                    (*joint_position_)(i) = joint_handles_[i].getPosition();
                    (*joint_velocity_)(i) = joint_handles_[i].getVelocity();
                    (*joint_acceleration_)(i) = acceleration; 
                }
                                
                // Compute Dynamics 
                int ret = id_solver_->CartToJnt(*joint_position_, 
                                                *joint_velocity_, 
                                                *joint_acceleration_, 
                                                *joint_wrenches_,
                                                *joint_effort_est_);
                if (ret < 0) { 
                    RCLCPP_ERROR(nh_.node()->get_logger(), "KDL: inverse dynamics ERROR");
                    realtime_pub_->unlock();
                    return;
                }
                                
                realtime_pub_->msg_.header.stamp = time;
                for (unsigned i=0; i<joint_handles_.size(); i++) {
                    realtime_pub_->msg_.est_ext_torques[i] = joint_handles_[i].getEffort() - (*joint_effort_est_)(i);
                }
                
                // Compute cartesian wrench on end effector
                ret = jac_solver_->JntToJac(*joint_position_, *jacobian_);
                if (ret < 0) { 
                    RCLCPP_ERROR(nh_.node()->get_logger(), "KDL: jacobian computation ERROR");
                    realtime_pub_->unlock();
                    return;
                }
                
                Eigen::MatrixXd jinv;
                pseudo_inverse(jacobian_->data.transpose(),jinv,true);                
                
                KDL::Wrench wrench;
                
                for (unsigned int i = 0; i < 6; i++) 
                    for (unsigned int j = 0; j < kdl_chain_.getNrOfJoints(); j++)
                        wrench[i] += jinv(i,j) * realtime_pub_->msg_.est_ext_torques[j];
                    
                lwr_controllers::wrenchKDLToMsg(wrench, realtime_pub_->msg_.est_ee_wrench_base);        
                    
                // Transform cartesian wrench into tool reference frame
                KDL::Frame tool_frame;
                fk_solver_->JntToCart(*joint_position_, tool_frame);
                KDL::Wrench tool_wrench = tool_frame * wrench;
                
                lwr_controllers::wrenchKDLToMsg(tool_wrench, realtime_pub_->msg_.est_ee_wrench);                
                
                realtime_pub_->unlockAndPublish();
            }
        }   
    }
    
    void ArmStateController::stopping(const rclcpp::Time& time)
    {}
    
}

PLUGINLIB_EXPORT_CLASS(arm_state_controller::ArmStateController, controller_interface::ControllerInterface)
