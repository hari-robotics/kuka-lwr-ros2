#include "lwr_hw/lwr_hw_gazebo.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/filters.hpp>
#include <angles/angles.h>
#include <iostream>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>

namespace lwr_hw
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
bool LWRHWGazebo::initSim(rclcpp::Node::SharedPtr &node,
  std::map<std::string, sim::Entity> &joints, const hardware_interface::HardwareInfo &info,
  sim::EntityComponentManager &ecm, int &)
{
  nh_=node; ecm_=&ecm;
  if (on_init(info) != CallbackReturn::SUCCESS) return false;
  sim_joints_.clear();
  for (const auto &name : buffers_.joint_names_) {
    const auto found=joints.find(name);
    if (found == joints.end()) return false;
    sim_joints_.push_back(found->second);
    if (!ecm.Component<sim::components::JointPosition>(found->second))
      ecm.CreateComponent(found->second,sim::components::JointPosition());
    if (!ecm.Component<sim::components::JointVelocity>(found->second))
      ecm.CreateComponent(found->second,sim::components::JointVelocity());
    if (!ecm.Component<sim::components::JointForce>(found->second))
      ecm.CreateComponent(found->second,sim::components::JointForce());
  }
  return true;
}
CallbackReturn LWRHWGazebo::on_init(const hardware_interface::HardwareInfo &info)
{
  SystemInterface::on_init(info);
  return buffers_.on_init(info);
}
CallbackReturn LWRHWGazebo::on_configure(const rclcpp_lifecycle::State &) {return CallbackReturn::SUCCESS;}
CallbackReturn LWRHWGazebo::on_activate(const rclcpp_lifecycle::State &state)
{
  auto result=buffers_.on_activate(state); active_=result==CallbackReturn::SUCCESS; return result;
}
CallbackReturn LWRHWGazebo::on_deactivate(const rclcpp_lifecycle::State &state)
{
  active_=false;
  if (ecm_) for (auto entity:sim_joints_)
    ecm_->SetComponentData<sim::components::JointForceCmd>(entity,{0.0});
  return buffers_.on_deactivate(state);
}
std::vector<hardware_interface::StateInterface> LWRHWGazebo::export_state_interfaces()
  {return buffers_.export_state_interfaces();}
std::vector<hardware_interface::CommandInterface> LWRHWGazebo::export_command_interfaces()
  {return buffers_.export_command_interfaces();}
return_type LWRHWGazebo::prepare_command_mode_switch(const std::vector<std::string> &start,
  const std::vector<std::string> &stop) {return buffers_.prepare_command_mode_switch(start,stop);}
return_type LWRHWGazebo::perform_command_mode_switch(const std::vector<std::string> &start,
  const std::vector<std::string> &stop) {return buffers_.perform_command_mode_switch(start,stop);}
return_type LWRHWGazebo::read(const rclcpp::Time &, const rclcpp::Duration &period)
{
  if (!ecm_ || period.nanoseconds() <= 0) return return_type::OK;
  for (size_t j=0; j<sim_joints_.size(); ++j) {
    const auto position=ecm_->Component<sim::components::JointPosition>(sim_joints_[j]);
    if (!position || position->Data().empty()) continue;
    buffers_.joint_position_prev_[j]=buffers_.joint_position_[j];
    buffers_.joint_position_[j]+=angles::shortest_angular_distance(
      buffers_.joint_position_[j],position->Data()[0]);
    buffers_.joint_position_kdl_(j)=buffers_.joint_position_[j];
    buffers_.joint_velocity_[j]=filters::exponentialSmoothing(
      (buffers_.joint_position_[j]-buffers_.joint_position_prev_[j])/period.seconds(),
      buffers_.joint_velocity_[j],0.2);
    const auto force=ecm_->Component<sim::components::JointForce>(sim_joints_[j]);
    if (force && !force->Data().empty()) buffers_.joint_effort_[j]=force->Data()[0];
    buffers_.joint_stiffness_[j]=buffers_.joint_stiffness_command_[j];
  }
  return return_type::OK;
}
return_type LWRHWGazebo::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  if (!active_ || !ecm_ || period.nanoseconds() <= 0) return return_type::OK;
  if (!buffers_.enforceLimits(period)) return return_type::ERROR;
  switch (buffers_.getControlStrategy())
  {
    case LWRHW::JOINT_POSITION:
      for (size_t j=0;j<sim_joints_.size();++j) {
        // Gazebo Classic SetPosition(): reset the joint position, no force command
        ecm_->SetComponentData<sim::components::JointForceCmd>(sim_joints_[j],{0.0});
        ecm_->SetComponentData<sim::components::JointPositionReset>(sim_joints_[j],
          {buffers_.joint_position_command_[j]});
      }
      break;

    case LWRHW::CARTESIAN_IMPEDANCE:
    {
      if(time.seconds()-lastT_ < ts_.seconds())
          break;
      lastT_ = time.seconds();
      RCLCPP_WARN(nh_->get_logger(), "CARTESIAN IMPEDANCE NOT AVAILABLE IN GAZEBO, PRINTING THE COMMANDED VALUES:");
      std::cout << "Notice that this printing is done only every " << ts_.seconds() << " seconds: to change this, change ts_ in lwr_hw_gazebo.hpp..." << std::endl;
      std::cout << "cart_pos_command_ = | ";
      for(int i=0; i < 12; ++i)
          std::cout << buffers_.cart_pos_command_[i] << " | ";
      std::cout << std::endl << "cart_stiff_command_ = | ";
      for(int i=0; i < 6; i++)
          std::cout << buffers_.cart_stiff_command_[i] << " | ";
      std::cout << std::endl << "cart_damp_command_ = | ";
      for(int i=0; i < 6; i++)
          std::cout << buffers_.cart_damp_command_[i] << " | ";
      std::cout << std::endl << "cart_wrench_command_ = | ";
      for(int i=0; i < 6; i++)
          std::cout << buffers_.cart_wrench_command_[i] << " | ";
      std::cout << std::endl << "Here, the call to doCartesianImpedanceControl() is done" << std::endl;
      break;
    }

    case LWRHW::JOINT_IMPEDANCE:
      // compute the gracity term
      buffers_.f_dyn_solver_->JntToGravity(buffers_.joint_position_kdl_,buffers_.gravity_effort_);
      for (size_t j=0;j<sim_joints_.size();++j) {
        // replicate the joint impedance control strategy
        // tau = k (q_FRI - q_msr) + tau_FRI + D(q_msr) + f_dyn(q_msr)
        const double stiffness_effort = 0.0;//10.0*( joint_position_command_[j] - joint_position_[j] ); // joint_stiffness_command_[j]*( joint_position_command_[j] - joint_position_[j] );
        //double damping_effort = joint_damping_command_[j]*( joint_velocity_[j] );
        const double effort = stiffness_effort + buffers_.joint_effort_command_[j] + buffers_.gravity_effort_(j);
        ecm_->SetComponentData<sim::components::JointForceCmd>(sim_joints_[j],{effort});
      }
      break;

    case LWRHW::GRAVITY_COMPENSATION:
      RCLCPP_WARN(nh_->get_logger(), "CARTESIAN IMPEDANCE NOT IMPLEMENTED");
      break;

    default:
      break;
  }
  return return_type::OK;
}
}
PLUGINLIB_EXPORT_CLASS(lwr_hw::LWRHWGazebo,gz_ros2_control::GazeboSimSystemInterface)
