#ifndef LWR_HW__LWR_HW_H
#define LWR_HW__LWR_HW_H

#include <hardware_interface/system_interface.hpp>
#include <joint_limits/joint_limits.hpp>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <set>
#include <vector>

namespace lwr_hw
{
// The original buffers and control strategies are shared by FRI, FRIL and Gazebo.
// ros2_control owns the update loop; no RobotHW handles or transmission parsing remain.
class LWRHW : public hardware_interface::SystemInterface
{
public:
  enum ControlStrategy {JOINT_POSITION = 10, CARTESIAN_IMPEDANCE = 20,
    JOINT_IMPEDANCE = 30, JOINT_EFFORT = 40, JOINT_STIFFNESS = 50, GRAVITY_COMPENSATION = 90};
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;

  virtual bool init() = 0;
  virtual bool stop() = 0;
  virtual bool doSwitch(ControlStrategy strategy);
  void setControlStrategy(ControlStrategy strategy) {current_strategy_ = strategy;}
  ControlStrategy getControlStrategy() const {return current_strategy_;}
  void reset();
  void resetCommands();
  bool enforceLimits(const rclcpp::Duration &period);
  void pollEmergencyStop();
  bool emergencyStopped() const {return emergency_stop_;}
  std::string parameter(const std::string &name, const std::string &fallback = "") const;

  std::string robot_namespace_, urdf_string_;
  urdf::Model urdf_model_;
  int n_joints_ = 7;
  ControlStrategy current_strategy_ = JOINT_POSITION;
  std::vector<std::string> joint_names_, cart_12_names_, cart_6_names_;
  std::vector<double> joint_lower_limits_, joint_upper_limits_, joint_effort_limits_,
    joint_lower_limits_stiffness_, joint_upper_limits_stiffness_,
    joint_lower_limits_damping_, joint_upper_limits_damping_;
  std::vector<double> joint_position_, joint_position_prev_, joint_velocity_, joint_effort_,
    joint_stiffness_, joint_damping_, joint_position_command_, joint_set_point_command_,
    joint_velocity_command_, joint_stiffness_command_, joint_damping_command_, joint_effort_command_,
    cart_pos_, cart_stiff_, cart_damp_, cart_wrench_, cart_pos_command_, cart_stiff_command_,
    cart_damp_command_, cart_wrench_command_;
  KDL::Chain lwr_chain_;
  std::unique_ptr<KDL::ChainDynParam> f_dyn_solver_;
  KDL::JntArray joint_position_kdl_, gravity_effort_;
  KDL::Vector gravity_;

protected:
  bool active_ = false;
  std::vector<joint_limits::JointLimits> limits_;
  std::vector<joint_limits::SoftJointLimits> soft_limits_;
  std::vector<bool> has_soft_limits_;
  std::vector<double> previous_position_command_;
  std::set<std::string> active_commands_;
  std::set<std::string> available_commands_;
  bool modeFor(const std::set<std::string> &, ControlStrategy &) const;
  bool nextCommands(const std::vector<std::string> &, const std::vector<std::string> &,
    std::set<std::string> &, ControlStrategy &) const;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  bool emergency_stop_ = false;
};
}
#endif
