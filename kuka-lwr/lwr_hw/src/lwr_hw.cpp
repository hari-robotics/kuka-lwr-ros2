#include "lwr_hw/lwr_hw.h"
#include <joint_limits/joint_limits_urdf.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace lwr_hw
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
// joint_limits_interface::internal::saturate()
double saturate(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

// Values of the stiffness metadata joints in kuka_lwr.urdf.xacro (min_stiffness,
// max_stiffness, rate_stiffness). They are used when the model is expanded with
// use_stiffness_joints:=false, which ros2_control requires.
constexpr double kMinStiffness = 0.0;
constexpr double kMaxStiffness = 5000.0;
constexpr double kRateStiffness = 20000.0;
}

std::string LWRHW::parameter(const std::string &name, const std::string &fallback) const
{
  const auto found = info_.hardware_parameters.find(name);
  return found == info_.hardware_parameters.end() ? fallback : found->second;
}

CallbackReturn LWRHW::on_init(const hardware_interface::HardwareInfo &info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  try {
    // SET NAME AND MODEL
    robot_namespace_ = parameter("name", "lwr");
    urdf_string_ = info.original_xml;
    std::cout << "Creating a KUKA LWR 4+ called: " << robot_namespace_ << std::endl;
    if (info.joints.size() != static_cast<size_t>(n_joints_) || !urdf_model_.initString(urdf_string_))
      throw std::runtime_error("Expected a seven-joint LWR and its URDF");

    // JOINT NAMES ARE TAKEN FROM URDF NAME CONVENTION
    joint_names_.clear(); cart_12_names_.clear(); cart_6_names_.clear();
    joint_names_.push_back( robot_namespace_ + std::string("_a1_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a2_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_e1_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a3_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a4_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a5_joint") );
    joint_names_.push_back( robot_namespace_ + std::string("_a6_joint") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xx") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yx") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zx") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_pos_x") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xy") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yy") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zy") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_pos_y") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xz") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yz") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zz") );
    cart_12_names_.push_back( robot_namespace_ + std::string("_pos_z") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_X") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_Y") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_Z") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_A") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_B") );
    cart_6_names_.push_back( robot_namespace_ + std::string("_C") );

    // VARIABLES
    for (auto *values : {&joint_position_, &joint_position_prev_, &joint_velocity_, &joint_effort_,
      &joint_est_ext_trq_, &joint_stiffness_, &joint_damping_, &joint_position_command_, &joint_set_point_command_,
      &joint_velocity_command_, &joint_effort_command_, &joint_stiffness_command_, &joint_damping_command_,
      &joint_lower_limits_, &joint_upper_limits_, &joint_effort_limits_, &joint_lower_limits_stiffness_,
      &joint_upper_limits_stiffness_, &joint_lower_limits_damping_, &joint_upper_limits_damping_})
      values->assign(n_joints_, 0.0);
    cart_pos_.resize(12); cart_pos_command_.resize(12);
    for (auto *values : {&cart_stiff_, &cart_damp_, &cart_wrench_, &cart_stiff_command_,
      &cart_damp_command_, &cart_wrench_command_}) values->assign(6, 0.0);

    // RESET VARIABLES
    reset();

    // registerJointLimits(): the limits are retrieved from the urdf_model
    limit_handles_.assign(n_joints_, JointLimitHandles{});
    for (int j = 0; j < n_joints_; ++j) {
      const auto resource = std::find_if(info.joints.begin(), info.joints.end(),
        [&](const auto &item) {return item.name == joint_names_[j];});
      if (resource == info.joints.end())
        throw std::runtime_error("Missing ros2_control joint: " + joint_names_[j]);
      for (const auto &interface : resource->command_interfaces)
        if (interface.name != "position" && interface.name != "effort" && interface.name != "stiffness" &&
            interface.name != "damping" && interface.name != "set_point")
          throw std::runtime_error("Unsupported LWR command interface: " + interface.name);

      auto &handles = limit_handles_[j];
      joint_lower_limits_[j] = -std::numeric_limits<double>::max();
      joint_upper_limits_[j] = std::numeric_limits<double>::max();
      joint_lower_limits_stiffness_[j] = -std::numeric_limits<double>::max();
      joint_upper_limits_stiffness_[j] = std::numeric_limits<double>::max();
      joint_lower_limits_damping_[j] = -std::numeric_limits<double>::max();
      joint_upper_limits_damping_[j] = std::numeric_limits<double>::max();
      joint_effort_limits_[j] = std::numeric_limits<double>::max();

      const auto urdf_joint = urdf_model_.getJoint(joint_names_[j]);
      if (urdf_joint) {
        handles.has_limits = joint_limits::getJointLimits(urdf_joint, handles.limits);
        handles.has_soft_limits = joint_limits::getSoftJointLimits(urdf_joint, handles.soft_limits);
        const auto urdf_joint_stiffness = urdf_model_.getJoint(joint_names_[j] + "_stiffness");
        if (urdf_joint_stiffness) {
          handles.has_limits_stiffness = joint_limits::getJointLimits(urdf_joint_stiffness, handles.limits_stiffness);
        } else {
          handles.limits_stiffness.has_position_limits = true;
          handles.limits_stiffness.min_position = kMinStiffness;
          handles.limits_stiffness.max_position = kMaxStiffness;
          handles.limits_stiffness.has_velocity_limits = true;
          handles.limits_stiffness.max_velocity = kRateStiffness;
          handles.has_limits_stiffness = true;
        }
        // there are no damping joints in the model, so damping is not limited
      }
      if (!handles.has_limits) {
        handles.has_soft_limits = false;
        handles.has_limits_stiffness = false;
        continue;
      }
      if (handles.limits.has_position_limits) {
        joint_lower_limits_[j] = handles.limits.min_position;
        joint_upper_limits_[j] = handles.limits.max_position;
      }
      if (handles.limits.has_effort_limits)
        joint_effort_limits_[j] = handles.limits.max_effort;
      if (handles.has_limits_stiffness && handles.limits_stiffness.has_position_limits) {
        joint_lower_limits_stiffness_[j] = handles.limits_stiffness.min_position;
        joint_upper_limits_stiffness_[j] = handles.limits_stiffness.max_position;
      }
    }

    // INIT KDL STUFF
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree))
      throw std::runtime_error("Failed to construct kdl tree");
    std::cout << "LWR kinematic successfully parsed with "
              << kdl_tree.getNrOfJoints()
              << " joints, and "
              << kdl_tree.getNrOfJoints()
              << " segments." << std::endl;
    std::string root_name = parameter("root");
    if (root_name.empty())
      root_name = kdl_tree.getRootSegment()->first; // default
    std::string tip_name = parameter("tip");
    if (tip_name.empty())
      tip_name = robot_namespace_ + std::string("_7_link"); // default
    std::cout << "Using root: " << root_name << " and tip: " << tip_name << std::endl;

    // this depends on how the world frame is set, in all our setups, world has always positive z pointing up.
    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;

    // Extract the chain from the tree
    if (!kdl_tree.getChain(root_name, tip_name, lwr_chain_))
      throw std::runtime_error("Failed to get KDL chain from tree");
    RCLCPP_INFO(rclcpp::get_logger("lwr_hw"), "Number of segments: %d", lwr_chain_.getNrOfSegments());
    RCLCPP_INFO(rclcpp::get_logger("lwr_hw"), "Number of joints in chain: %d", lwr_chain_.getNrOfJoints());
    f_dyn_solver_ = std::make_unique<KDL::ChainDynParam>(lwr_chain_, gravity_);
    joint_position_kdl_ = KDL::JntArray(lwr_chain_.getNrOfJoints());
    gravity_effort_ = KDL::JntArray(lwr_chain_.getNrOfJoints());

    // ROS 1 advertised with a NodeHandle member of LWRHW at the end of create(); the
    // hardware node is created here so that the Gazebo plugin (no on_configure) has it too
    if (!node_ && rclcpp::ok())
      node_ = std::make_shared<rclcpp::Node>(robot_namespace_ + "_hw", "/" + robot_namespace_);
    if (node_)
      pub_estExtTrq_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/est_ext_torque", 10); //ADDED 10.22

    std::cout << "Succesfully created an abstract LWR 4+ ARM with interfaces to ROS control" << std::endl;
    return CallbackReturn::SUCCESS;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("lwr_hw"), "%s", error.what());
    return CallbackReturn::ERROR;
  }
}

CallbackReturn LWRHW::on_configure(const rclcpp_lifecycle::State &)
{
  // advertise the e-stop topic (ROS 1: subscribed by lwr_hw_fri_node in its namespace)
  if (node_ && !emergency_stop_sub_) {
    emergency_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>("emergency_stop", 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr e_stop_msg) {emergency_stop_ = e_stop_msg->data;});
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn LWRHW::on_activate(const rclcpp_lifecycle::State &)
{
  if (!init()) return CallbackReturn::ERROR;
  return CallbackReturn::SUCCESS;
}

CallbackReturn LWRHW::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LWRHW::on_cleanup(const rclcpp_lifecycle::State &)
{
  emergency_stop_sub_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn LWRHW::on_shutdown(const rclcpp_lifecycle::State &state)
{
  stop();
  return on_cleanup(state);
}

CallbackReturn LWRHW::on_error(const rclcpp_lifecycle::State &state)
{
  stop();
  return on_cleanup(state);
}

void LWRHW::pollEmergencyStop()
{
  if (node_) rclcpp::spin_some(node_);
  if (emergency_stop_ && !emergency_stop_handled_)
  {
    // same messages as ROS 1 lwr_hw_fri_node when the e-stop is pressed
    RCLCPP_WARN(rclcpp::get_logger("lwr_hw"), "E-STOP HAS BEEN PRESSED: Controllers will be restarted, but the robot won't move until you release the E-Stop");
    RCLCPP_WARN(rclcpp::get_logger("lwr_hw"), "HOW TO RELEASE E-STOP: ros2 topic pub -r 10 /NAMESPACE/emergency_stop std_msgs/msg/Bool 'data: false'");
  }
  emergency_stop_handled_ = emergency_stop_;
  if (emergency_stop_) semanticZero();
}

  // reset values
  void LWRHW::reset()
  {
    for (int j = 0; j < n_joints_; ++j)
    {
      joint_position_[j] = 0.0;
      joint_position_prev_[j] = 0.0;
      joint_velocity_[j] = 0.0;
      joint_effort_[j] = 0.0;
      joint_stiffness_[j] = 0.0;
      joint_damping_[j] = 0.0;

      joint_position_command_[j] = 0.0;
      joint_set_point_command_[j] = 0.0;
      joint_velocity_command_[j] = 0.0;
      joint_effort_command_[j] = 0.0;
      joint_stiffness_command_[j] = 1000.0;
      joint_damping_command_[j] = 0.7;
    }

    for(int i=0; i < 12; ++i)
    {
      cart_pos_[i] = 0.0;
      cart_pos_command_[i] = 0.0;
    }
    cart_pos_[0] = 1.0;
    cart_pos_[5] = 1.0;
    cart_pos_[10] = 1.0;
    cart_pos_command_[0] = 1.0;
    cart_pos_command_[5] = 1.0;
    cart_pos_command_[10] = 1.0;
    for(int i=0; i < 3; i++)
    {
      cart_stiff_[i] = 0.0;
      cart_stiff_[i + 3] = 0.0;
      cart_damp_[i] = 0.0;
      cart_damp_[i + 3] = 0.0;
      cart_wrench_[i] = 0.0;
      cart_wrench_[i + 3] = 0.0;
      cart_stiff_command_[i] = 800;
      cart_stiff_command_[i + 3] = 50;
      cart_damp_command_[i] = 10;
      cart_damp_command_[i + 3] = 1;
      cart_wrench_command_[i] = 0.0;
      cart_wrench_command_[i + 3] = 0.0;
    }

    current_strategy_ = JOINT_POSITION;

    return;
  }

std::vector<hardware_interface::StateInterface> LWRHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (const auto &joint : info_.joints) {
    auto index = std::find(joint_names_.begin(), joint_names_.end(), joint.name) - joint_names_.begin();
    if (index == static_cast<long>(joint_names_.size())) throw std::runtime_error("Unknown LWR joint");
    for (const auto &interface : joint.state_interfaces) {
      auto *values = interface.name == "position" ? &joint_position_ :
        interface.name == "velocity" ? &joint_velocity_ : interface.name == "effort" ? &joint_effort_ :
        interface.name == "stiffness" ? &joint_stiffness_ : &joint_damping_;
      interfaces.emplace_back(joint.name, interface.name, &values->at(index));
    }
  }
  // Now for cart variables
  for (size_t j = 0; j < 12; ++j) interfaces.emplace_back(cart_12_names_[j], "position", &cart_pos_[j]);
  for (size_t j = 0; j < 6; ++j) {
    interfaces.emplace_back(cart_6_names_[j]+"_stiffness", "position", &cart_stiff_[j]);
    interfaces.emplace_back(cart_6_names_[j]+"_damping", "position", &cart_damp_[j]);
    interfaces.emplace_back(cart_6_names_[j]+"_wrench", "position", &cart_wrench_[j]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> LWRHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (const auto &joint : info_.joints) {
    auto index = std::find(joint_names_.begin(), joint_names_.end(), joint.name) - joint_names_.begin();
    if (index == static_cast<long>(joint_names_.size())) throw std::runtime_error("Unknown LWR joint");
    for (const auto &interface : joint.command_interfaces) {
      auto *values = interface.name == "position" ? &joint_position_command_ :
        interface.name == "effort" ? &joint_effort_command_ : interface.name == "stiffness" ?
        &joint_stiffness_command_ : interface.name == "damping" ? &joint_damping_command_ : &joint_set_point_command_;
      interfaces.emplace_back(joint.name, interface.name, &values->at(index));
    }
  }
  // Now for cart variables
  for (size_t j = 0; j < 12; ++j) interfaces.emplace_back(cart_12_names_[j], "position", &cart_pos_command_[j]);
  for (size_t j = 0; j < 6; ++j) {
    interfaces.emplace_back(cart_6_names_[j]+"_stiffness", "position", &cart_stiff_command_[j]);
    interfaces.emplace_back(cart_6_names_[j]+"_damping", "position", &cart_damp_command_[j]);
    interfaces.emplace_back(cart_6_names_[j]+"_wrench", "position", &cart_wrench_command_[j]);
  }
  return interfaces;
}

bool LWRHW::isOwnInterface(const std::string &name) const
{
  const auto resource = name.substr(0, name.rfind('/'));
  if (std::find(joint_names_.begin(), joint_names_.end(), resource) != joint_names_.end() ||
      std::find(cart_12_names_.begin(), cart_12_names_.end(), resource) != cart_12_names_.end())
    return true;
  for (const auto &axis : cart_6_names_)
    for (const auto &kind : {"_stiffness", "_damping", "_wrench"})
      if (resource == axis + kind) return true;
  return false;
}

// ROS 1 registered position commands in PositionJointInterface, effort, set_point, stiffness
// and damping commands in EffortJointInterface and the cartesian variables in
// PositionCartesianInterface. The interface families below follow that registration.
LWRHW::ControlStrategy LWRHW::getNewControlStrategy(const std::vector<std::string> &start_list,
  ControlStrategy default_control_strategy) const
{
  ControlStrategy desired_strategy = default_control_strategy;
  // NOTE that this allows to switch only based on the strategy of the first controller
  for (const auto &name : start_list)
  {
    if (!isOwnInterface(name)) continue;
    const auto resource = name.substr(0, name.rfind('/'));
    const auto interface = name.substr(name.rfind('/') + 1);
    if (std::find(joint_names_.begin(), joint_names_.end(), resource) == joint_names_.end())
    {
      std::cout << "Request to switch to hardware_interface::PositionCartesianInterface (CARTESIAN_IMPEDANCE)" << std::endl;
      desired_strategy = CARTESIAN_IMPEDANCE;
      break;
    }
    if (interface == "position")
    {
      std::cout << "Request to switch to hardware_interface::PositionJointInterface (JOINT_POSITION)" << std::endl;
      desired_strategy = JOINT_POSITION;
      break;
    }
    std::cout << "Request to switch to hardware_interface::EffortJointInterface (JOINT_IMPEDANCE)" << std::endl;
    desired_strategy = JOINT_IMPEDANCE;
    break;
  }
  return desired_strategy;
}

return_type LWRHW::prepare_command_mode_switch(const std::vector<std::string> &start_list,
  const std::vector<std::string> &)
{
  int counter_position = 0, counter_effort = 0, counter_cartesian = 0;
  for (const auto &name : start_list)
  {
    if (!isOwnInterface(name)) continue;
    const auto resource = name.substr(0, name.rfind('/'));
    const auto interface = name.substr(name.rfind('/') + 1);
    if (std::find(joint_names_.begin(), joint_names_.end(), resource) == joint_names_.end())
      counter_cartesian = 1;
    else if (interface == "position")
      counter_position = 1;
    else
      counter_effort = 1;
  }

  if( (counter_position+counter_effort+counter_cartesian)>1)
  {
    std::cout << "OOPS! Currently we are using the JointCommandInterface to switch mode, this is not strictly correct. "
    << "This is temporary until a joint_mode_controller is available (so you can have different interfaces available in different modes)"
    << "Having said this, we do not support more than one controller that ones to act on any given JointCommandInterface"
    << "and we can't switch"
    << std::endl;
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type LWRHW::perform_command_mode_switch(const std::vector<std::string> &start_list,
  const std::vector<std::string> &stop_list)
{
  // only switches that touch this arm reach ROS 1 doSwitch()
  if (std::none_of(start_list.begin(), start_list.end(), [this](const auto &n) {return isOwnInterface(n);}) &&
      std::none_of(stop_list.begin(), stop_list.end(), [this](const auto &n) {return isOwnInterface(n);}))
    return return_type::OK;
  // at this point, we now that there is only one controller that ones to command joints
  ControlStrategy desired_strategy = JOINT_POSITION; // default
  desired_strategy = getNewControlStrategy(start_list, desired_strategy);
  doSwitch(desired_strategy);
  return return_type::OK;
}

void LWRHW::semanticZero()
{
  for (int j = 0; j < n_joints_; ++j)
  {
    ///semantic Zero
    joint_position_command_[j] = joint_position_[j];
    joint_effort_command_[j] = 0.0;

    ///reset joint_limit_interfaces
    limit_handles_[j].prev_position_cmd = std::numeric_limits<double>::quiet_NaN();
  }
}

void LWRHW::doSwitch(ControlStrategy desired_strategy)
{
  semanticZero();

  if(desired_strategy == getControlStrategy())
  {
    std::cout << "The ControlStrategy didn't change, it is already: " << getControlStrategy() << std::endl;
  }
  else
  {
    setControlStrategy(desired_strategy);
    std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
  }
}

// ROS 1 enforceLimits(): the joint_limits_interface saturation / soft limits handles
// registered in registerJointLimits(), with the same equations.
bool LWRHW::enforceLimits(const rclcpp::Duration &period)
{
  const double dt = period.seconds();
  for (int j = 0; j < n_joints_; ++j)
  {
    auto &h = limit_handles_[j];
    if (h.has_limits)
    {
      const auto &limits = h.limits;
      const double max_vel_limit = limits.has_velocity_limits ? limits.max_velocity : std::numeric_limits<double>::max();
      if (h.has_soft_limits)
      {
        const auto &soft_limits = h.soft_limits;
        // EffortJointSoftLimitsHandle
        {
          const double pos = joint_position_[j];
          const double vel = joint_velocity_[j];
          double soft_min_vel = -max_vel_limit, soft_max_vel = max_vel_limit;
          if (limits.has_position_limits)
          {
            soft_min_vel = saturate(-soft_limits.k_position * (pos - soft_limits.min_position), -max_vel_limit, max_vel_limit);
            soft_max_vel = saturate(-soft_limits.k_position * (pos - soft_limits.max_position), -max_vel_limit, max_vel_limit);
          }
          const double soft_min_eff = saturate(-soft_limits.k_velocity * (vel - soft_min_vel), -limits.max_effort, limits.max_effort);
          const double soft_max_eff = saturate(-soft_limits.k_velocity * (vel - soft_max_vel), -limits.max_effort, limits.max_effort);
          joint_effort_command_[j] = saturate(joint_effort_command_[j], soft_min_eff, soft_max_eff);
        }
        // PositionJointSoftLimitsHandle
        {
          if (std::isnan(h.prev_position_cmd)) h.prev_position_cmd = joint_position_[j];
          const double pos = h.prev_position_cmd;
          double soft_min_vel = -max_vel_limit, soft_max_vel = max_vel_limit;
          if (limits.has_position_limits)
          {
            soft_min_vel = saturate(-soft_limits.k_position * (pos - soft_limits.min_position), -max_vel_limit, max_vel_limit);
            soft_max_vel = saturate(-soft_limits.k_position * (pos - soft_limits.max_position), -max_vel_limit, max_vel_limit);
          }
          double pos_low = pos + soft_min_vel * dt;
          double pos_high = pos + soft_max_vel * dt;
          if (limits.has_position_limits)
          {
            pos_low = std::max(pos_low, limits.min_position);
            pos_high = std::min(pos_high, limits.max_position);
          }
          joint_position_command_[j] = saturate(joint_position_command_[j], pos_low, pos_high);
          h.prev_position_cmd = joint_position_command_[j];
        }
      }
      else
      {
        // EffortJointSaturationHandle
        if (limits.has_effort_limits)
          joint_effort_command_[j] = saturate(joint_effort_command_[j], -limits.max_effort, limits.max_effort);
        // PositionJointSaturationHandle
        {
          if (std::isnan(h.prev_position_cmd)) h.prev_position_cmd = joint_position_[j];
          double min_pos = limits.has_position_limits ? limits.min_position : -std::numeric_limits<double>::max();
          double max_pos = limits.has_position_limits ? limits.max_position : std::numeric_limits<double>::max();
          if (limits.has_velocity_limits)
          {
            const double delta_pos = limits.max_velocity * dt;
            min_pos = std::max(h.prev_position_cmd - delta_pos, min_pos);
            max_pos = std::min(h.prev_position_cmd + delta_pos, max_pos);
          }
          joint_position_command_[j] = saturate(joint_position_command_[j], min_pos, max_pos);
          h.prev_position_cmd = joint_position_command_[j];
        }
      }
    }
    // sj_sat_interface_: PositionJointSaturationHandle on the stiffness handle
    if (h.has_limits_stiffness)
    {
      const auto &limits = h.limits_stiffness;
      if (std::isnan(h.prev_stiffness_cmd)) h.prev_stiffness_cmd = joint_stiffness_[j];
      double min_pos = limits.has_position_limits ? limits.min_position : -std::numeric_limits<double>::max();
      double max_pos = limits.has_position_limits ? limits.max_position : std::numeric_limits<double>::max();
      if (limits.has_velocity_limits)
      {
        const double delta_pos = limits.max_velocity * dt;
        min_pos = std::max(h.prev_stiffness_cmd - delta_pos, min_pos);
        max_pos = std::min(h.prev_stiffness_cmd + delta_pos, max_pos);
      }
      joint_stiffness_command_[j] = saturate(joint_stiffness_command_[j], min_pos, max_pos);
      h.prev_stiffness_cmd = joint_stiffness_command_[j];
    }
  }
//	ADDED 10.22 - start
//    for (int j = 0; j < n_joints_; ++j){
//    	ROS_INFO_STREAM(" joint_est_ext_trq_(" << j << ") = " << joint_est_ext_trq_[j]); //ADDED 10.22
//    }
	// TODO(ros2-migration): as in ROS 1, two layout dimensions are appended on every call,
	// so the message grows without bound (see MIGRATION_NOTES).
	msg_estExtTrq_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	msg_estExtTrq_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	msg_estExtTrq_.layout.dim[0].label = "height";
	msg_estExtTrq_.layout.dim[0].size = 7; //H
	msg_estExtTrq_.layout.dim[0].stride = 7*1; //H*W
	msg_estExtTrq_.layout.dim[1].label = "width";
	msg_estExtTrq_.layout.dim[1].size = 1; //W
	msg_estExtTrq_.layout.dim[1].stride = 1; //W
	msg_estExtTrq_.layout.data_offset = 0;
	std::vector<double> vec_estExtTrq_(1*7, 0); //vec(W*H, 0)
	for (int h=0; h<7; h++)
		for (int k=0; k<1; k++)
			vec_estExtTrq_[h*1 + k] = joint_est_ext_trq_[h]; //vec[i*W + j]
		msg_estExtTrq_.data = vec_estExtTrq_;
	
	// ROS 1 Publisher::publish() returned without serializing when nobody subscribed;
	// rclcpp serializes on every call, which with the growing layout above slows the
	// update loop down without any subscriber (measured in Gazebo). Keep the ROS 1 semantics.
	if (pub_estExtTrq_ && pub_estExtTrq_->get_subscription_count() > 0) pub_estExtTrq_->publish(msg_estExtTrq_);
//	ADDED 10.22 - end
  return true;
}
}
