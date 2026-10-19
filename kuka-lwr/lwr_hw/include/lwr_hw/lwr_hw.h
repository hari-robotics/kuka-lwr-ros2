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
#include "std_msgs/msg/float64_multi_array.hpp" //ADDED 10.22
#include <limits>
#include <memory>
#include <vector>

namespace lwr_hw
{
// The original buffers and control strategies are shared by FRI, FRIL and Gazebo.
// RobotHW handles/transmissions are replaced by ros2_control SystemInterface exports.
class LWRHW : public hardware_interface::SystemInterface
{
public:
  // control strategies
  // JOINT_POSITION -> strategy 10 -> triggered with PoitionJointInterface
  // CARTESIAN_IMPEDANCE -> strategy 20 (not implemented)
  // JOINT_IMPEDANCE -> strategy 30 -> triggered with (ToDo) ImpedanceJointInterface
  // JOINT_EFFORT -> strategy 40 with special configuration, triggered with EffortJointInterface
  // JOINT_STIFFNESS -> strategy 50 with special configuration, trigered with (ToDo) StiffnessJointInterface
  // GRAVITY_COMPENSATION -> (does not exist in the real robot, achieved with low stiffness)
  // __Note__: StiffnessJointInterface + EffortJointInterface = ImpedanceJointInterface
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
  // ROS 1 prepareSwitch()/doSwitch(start_list, stop_list)
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;

  /**
   * @brief Function to get the control strategy based on the list of command interfaces to be started
   *
   * @param start_list command interfaces to be started
   * @param default_control_strategy value to use as a default return value
   *
   * @return the output control strategy found based on the lists
   */
  ControlStrategy getNewControlStrategy(const std::vector<std::string> &start_list,
    ControlStrategy default_control_strategy = JOINT_POSITION) const;

  // This functions must be implemented depending on the outlet (Real FRI/FRIL, Gazebo, etc.)
  virtual bool init() = 0;
  virtual void stop() {}
  // Called with the strategy computed from the start list, as ROS 1 doSwitch() did.
  virtual void doSwitch(ControlStrategy desired_strategy);

  // get/set control method
  void setControlStrategy(ControlStrategy strategy) {current_strategy_ = strategy;}
  ControlStrategy getControlStrategy() const {return current_strategy_;}

  // Before write, you can use this function to enforce limits for all values
  bool enforceLimits(const rclcpp::Duration &period);

  // Set all members to default values
  void reset();

  std::string parameter(const std::string &name, const std::string &fallback = "") const;

  // Strings
  std::string robot_namespace_;

  // Model
  std::string urdf_string_;
  urdf::Model urdf_model_;

  ControlStrategy current_strategy_ = JOINT_POSITION;

  // configuration
  int n_joints_ = 7; // safe magic number, the kuka lwr 4+ has 7 joints
  std::vector<std::string> joint_names_;
  std::vector<std::string> cart_12_names_;
  std::vector<std::string> cart_6_names_;

  // limits
  std::vector<double>
  joint_lower_limits_,
  joint_upper_limits_,
  joint_effort_limits_,
  joint_lower_limits_stiffness_,
  joint_upper_limits_stiffness_,
  joint_lower_limits_damping_,
  joint_upper_limits_damping_;

  // state and commands
  std::vector<double>
  joint_position_,
  joint_position_prev_,
  joint_velocity_,
  joint_effort_,
  joint_est_ext_trq_,	//ADDED 10.22
  joint_stiffness_,
  joint_damping_,
  joint_position_command_,
  joint_set_point_command_,
  joint_velocity_command_,
  joint_stiffness_command_,
  joint_damping_command_,
  joint_effort_command_,
  cart_pos_,
  cart_stiff_,
  cart_damp_,
  cart_wrench_,
  cart_pos_command_,
  cart_stiff_command_,
  cart_damp_command_,
  cart_wrench_command_;

  // NOTE:
  // joint_velocity_command is not really to command the kuka arm in velocity,
  // since it doesn't have an interface for that
  // this is used to avoid speed limit error in the kuka controller by
  // computing a fake velocity command using the received position command and
  // the current position, without smoothing.

  // KDL stuff to compute ik, gravity term, etc.
  KDL::Chain lwr_chain_;
  std::unique_ptr<KDL::ChainDynParam> f_dyn_solver_;
  KDL::JntArray joint_position_kdl_, gravity_effort_;
  KDL::Vector gravity_;

protected:
  // ROS 1 "semantic zero" of doSwitch(): command the measured pose and zero effort,
  // and reset the position limit handles.
  void semanticZero();
  // TODO(ros2-migration): ROS 1 reset the controllers from the node main loop while
  // /<name>/emergency_stop was true; ros2_control has no hardware-side controller
  // reset, so the commands are held at the measured pose instead.
  void pollEmergencyStop();
  bool emergencyStopped() const {return emergency_stop_;}

  // joint_limits_interface handles of ROS 1, one entry per joint
  struct JointLimitHandles
  {
    joint_limits::JointLimits limits;            // effort / position saturation
    joint_limits::SoftJointLimits soft_limits;   // effort / position soft limits
    bool has_limits = false, has_soft_limits = false;
    joint_limits::JointLimits limits_stiffness;  // sj_sat_interface_
    bool has_limits_stiffness = false;
    double prev_position_cmd = std::numeric_limits<double>::quiet_NaN();
    double prev_stiffness_cmd = std::numeric_limits<double>::quiet_NaN();
  };
  std::vector<JointLimitHandles> limit_handles_;

  bool isOwnInterface(const std::string &name) const;

  rclcpp::Node::SharedPtr node_;
  //  Create nodehandle and publisher to publish estimated external torques
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_estExtTrq_;	//ADDED 10.22
  std_msgs::msg::Float64MultiArray msg_estExtTrq_; // ADDED 10.22
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  bool emergency_stop_ = false;
};
}
#endif
