#include "lwr_hw/lwr_hw_gazebo.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/filters.hpp>
#include <angles/angles.h>
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
    buffers_.joint_damping_[j]=buffers_.joint_damping_command_[j];
  }
  return return_type::OK;
}
return_type LWRHWGazebo::write(const rclcpp::Time &, const rclcpp::Duration &period)
{
  if (!active_ || !ecm_ || period.nanoseconds() <= 0) return return_type::OK;
  if (!buffers_.enforceLimits(period)) return return_type::ERROR;
  const auto strategy=buffers_.getControlStrategy();
  if (strategy==LWRHW::CARTESIAN_IMPEDANCE) {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(),*nh_->get_clock(),3000,
      "Cartesian FRI commands are available as metadata only, as in the ROS 1 Gazebo plugin");
    return return_type::OK;
  }
  if (strategy==LWRHW::JOINT_POSITION) {
    for (size_t j=0;j<sim_joints_.size();++j) {
      ecm_->SetComponentData<sim::components::JointForceCmd>(sim_joints_[j],{0.0});
      ecm_->SetComponentData<sim::components::JointPositionReset>(sim_joints_[j],
        {buffers_.joint_position_command_[j]});
    }
  } else if (strategy==LWRHW::JOINT_IMPEDANCE || strategy==LWRHW::JOINT_EFFORT) {
    if (buffers_.f_dyn_solver_->JntToGravity(buffers_.joint_position_kdl_,buffers_.gravity_effort_)<0)
      return return_type::ERROR;
    for (size_t j=0;j<sim_joints_.size();++j) {
      // Preserve the original law: stiffness term was disabled in ROS 1.
      const double stiffness_effort=0.0;
      const double effort=stiffness_effort+buffers_.joint_effort_command_[j]+buffers_.gravity_effort_(j);
      ecm_->SetComponentData<sim::components::JointForceCmd>(sim_joints_[j],{effort});
    }
  }
  return return_type::OK;
}
}
PLUGINLIB_EXPORT_CLASS(lwr_hw::LWRHWGazebo,gz_ros2_control::GazeboSimSystemInterface)
