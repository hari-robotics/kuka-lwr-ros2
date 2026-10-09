#include "lwr_hw/lwr_hw.h"
#include <joint_limits/joint_limits_urdf.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lwr_hw
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
std::string LWRHW::parameter(const std::string &name, const std::string &fallback) const
{
  const auto found = info_.hardware_parameters.find(name);
  return found == info_.hardware_parameters.end() ? fallback : found->second;
}

CallbackReturn LWRHW::on_init(const hardware_interface::HardwareInfo &info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  try {
    robot_namespace_ = parameter("name", "lwr");
    urdf_string_ = info.original_xml;
    if (robot_namespace_.empty() || robot_namespace_.find('/') != std::string::npos ||
        info.joints.size() != 7 || !urdf_model_.initString(urdf_string_))
      throw std::runtime_error("Expected a named seven-joint LWR and its original URDF");
    joint_names_.clear(); cart_12_names_.clear(); cart_6_names_.clear();
    for (const auto &suffix : {"a1", "a2", "e1", "a3", "a4", "a5", "a6"})
      joint_names_.push_back(robot_namespace_ + "_" + suffix + "_joint");
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

    for (auto *values : {&joint_position_, &joint_position_prev_, &joint_velocity_, &joint_effort_,
      &joint_stiffness_, &joint_damping_, &joint_position_command_, &joint_set_point_command_,
      &joint_velocity_command_, &joint_effort_command_, &joint_stiffness_command_, &joint_damping_command_,
      &joint_lower_limits_, &joint_upper_limits_, &joint_effort_limits_, &joint_lower_limits_stiffness_,
      &joint_upper_limits_stiffness_, &joint_lower_limits_damping_, &joint_upper_limits_damping_,
      &previous_position_command_}) values->assign(7, 0.0);
    cart_pos_.resize(12); cart_pos_command_.resize(12);
    for (auto *values : {&cart_stiff_, &cart_damp_, &cart_wrench_, &cart_stiff_command_,
      &cart_damp_command_, &cart_wrench_command_}) values->assign(6, 0.0);
    reset();
    limits_.resize(7); soft_limits_.resize(7); has_soft_limits_.assign(7, false);
    for (size_t j = 0; j < joint_names_.size(); ++j) {
      const auto resource = std::find_if(info.joints.begin(), info.joints.end(),
        [&](const auto &item) {return item.name == joint_names_[j];});
      const auto joint = urdf_model_.getJoint(joint_names_[j]);
      if (resource == info.joints.end() || !joint_limits::getJointLimits(joint, limits_[j]))
        throw std::runtime_error("Missing joint or limits: " + joint_names_[j]);
      has_soft_limits_[j] = joint_limits::getSoftJointLimits(joint, soft_limits_[j]);
      joint_lower_limits_[j] = limits_[j].min_position;
      joint_upper_limits_[j] = limits_[j].max_position;
      joint_effort_limits_[j] = limits_[j].max_effort;
      joint_lower_limits_stiffness_[j] = 0.0; joint_upper_limits_stiffness_[j] = 5000.0;
      joint_lower_limits_damping_[j] = 0.0; joint_upper_limits_damping_[j] = 1.0;
      for (const auto &interface : resource->command_interfaces)
        if (interface.name != "position" && interface.name != "effort" && interface.name != "stiffness" &&
            interface.name != "damping" && interface.name != "set_point")
          throw std::runtime_error("Unsupported LWR command interface: " + interface.name);
      for (const auto &interface : resource->state_interfaces)
        if (interface.name != "position" && interface.name != "velocity" && interface.name != "effort" &&
            interface.name != "stiffness" && interface.name != "damping")
          throw std::runtime_error("Unsupported LWR state interface: " + interface.name);
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model_, tree) || !tree.getChain(
        parameter("root", robot_namespace_ + "_base_link"),
        parameter("tip", robot_namespace_ + "_7_link"), lwr_chain_) || lwr_chain_.getNrOfJoints() != 7)
      throw std::runtime_error("Cannot construct the seven-joint LWR KDL chain");
    gravity_ = KDL::Vector(0, 0, -9.81);
    f_dyn_solver_ = std::make_unique<KDL::ChainDynParam>(lwr_chain_, gravity_);
    joint_position_kdl_.resize(7); gravity_effort_.resize(7);
    available_commands_.clear();
    for (const auto &interface : export_command_interfaces()) available_commands_.insert(interface.get_name());
    active_commands_.clear();
    return CallbackReturn::SUCCESS;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("lwr_hw"), "%s", error.what()); return CallbackReturn::ERROR;
  }
}

CallbackReturn LWRHW::on_configure(const rclcpp_lifecycle::State &)
{
  if (!node_ && rclcpp::ok()) {
    node_ = std::make_shared<rclcpp::Node>(robot_namespace_ + "_hw", "/" + robot_namespace_);
    emergency_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>("emergency_stop", 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {emergency_stop_ = message->data;});
  }
  return CallbackReturn::SUCCESS;
}
CallbackReturn LWRHW::on_activate(const rclcpp_lifecycle::State &)
{
  if (!init()) return CallbackReturn::ERROR;
  current_strategy_ = JOINT_POSITION;
  resetCommands(); active_ = true; return CallbackReturn::SUCCESS;
}
CallbackReturn LWRHW::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (!active_) return CallbackReturn::SUCCESS;
  resetCommands();
  const bool stopped = stop();
  active_ = !stopped;
  if (stopped) active_commands_.clear();
  return stopped ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}
CallbackReturn LWRHW::on_cleanup(const rclcpp_lifecycle::State &state)
{
  auto result = on_deactivate(state);
  emergency_stop_sub_.reset(); node_.reset(); return result;
}
CallbackReturn LWRHW::on_shutdown(const rclcpp_lifecycle::State &state) {return on_cleanup(state);}
CallbackReturn LWRHW::on_error(const rclcpp_lifecycle::State &state) {return on_cleanup(state);}
void LWRHW::pollEmergencyStop()
{
  if (node_) rclcpp::spin_some(node_);
  if (emergency_stop_) resetCommands();
}
void LWRHW::resetCommands()
{
  joint_position_command_ = joint_position_; joint_set_point_command_ = joint_position_;
  previous_position_command_ = joint_position_;
  std::fill(joint_effort_command_.begin(), joint_effort_command_.end(), 0.0);
  cart_pos_command_ = cart_pos_;
  std::fill(cart_wrench_command_.begin(), cart_wrench_command_.end(), 0.0);
}
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
  for (size_t j = 0; j < 12; ++j) interfaces.emplace_back(cart_12_names_[j], "position", &cart_pos_command_[j]);
  for (size_t j = 0; j < 6; ++j) {
    interfaces.emplace_back(cart_6_names_[j]+"_stiffness", "position", &cart_stiff_command_[j]);
    interfaces.emplace_back(cart_6_names_[j]+"_damping", "position", &cart_damp_command_[j]);
    interfaces.emplace_back(cart_6_names_[j]+"_wrench", "position", &cart_wrench_command_[j]);
  }
  return interfaces;
}

bool LWRHW::modeFor(const std::set<std::string> &commands, ControlStrategy &mode) const
{
  if (commands.empty()) {mode = JOINT_POSITION; return true;}
  size_t position=0, effort=0, stiffness=0, damping=0, set_point=0, cartesian=0;
  for (const auto &name : commands) {
    const auto slash = name.rfind('/');
    const auto resource = name.substr(0, slash), interface = name.substr(slash + 1);
    if (std::find(joint_names_.begin(), joint_names_.end(), resource) == joint_names_.end()) {++cartesian; continue;}
    if (interface == "position") ++position;
    else if (interface == "effort") ++effort;
    else if (interface == "stiffness") ++stiffness;
    else if (interface == "damping") ++damping;
    else if (interface == "set_point") ++set_point;
    else return false;
  }
  if (cartesian) {mode=CARTESIAN_IMPEDANCE; return cartesian==30 && commands.size()==30;}
  if (position) {mode=JOINT_POSITION; return position==7 && commands.size()==7;}
  if (stiffness || damping || set_point) {
    mode=JOINT_IMPEDANCE; return effort==7 && stiffness==7 && damping==7 && set_point==7;
  }
  mode=JOINT_EFFORT; return effort==7;
}
bool LWRHW::nextCommands(const std::vector<std::string> &start, const std::vector<std::string> &stop,
  std::set<std::string> &next, ControlStrategy &mode) const
{
  next=active_commands_;
  auto belongs=[&](const std::string &name) {
    const auto resource=name.substr(0,name.rfind('/'));
    if (std::find(joint_names_.begin(),joint_names_.end(),resource)!=joint_names_.end() ||
        std::find(cart_12_names_.begin(),cart_12_names_.end(),resource)!=cart_12_names_.end()) return true;
    for (const auto &axis:cart_6_names_)
      for (const auto &kind:{"_stiffness","_damping","_wrench"})
        if (resource==axis+kind) return true;
    return false;
  };
  for (const auto &name : stop) if (available_commands_.count(name)) next.erase(name);
  for (const auto &name : start) {
    if (!available_commands_.count(name)) {if (belongs(name)) return false; continue;}
    if (!next.insert(name).second) return false;
  }
  return modeFor(next,mode);
}
return_type LWRHW::prepare_command_mode_switch(const std::vector<std::string> &start,
  const std::vector<std::string> &stop)
{
  std::set<std::string> next; ControlStrategy mode;
  return nextCommands(start,stop,next,mode) ? return_type::OK : return_type::ERROR;
}
return_type LWRHW::perform_command_mode_switch(const std::vector<std::string> &start,
  const std::vector<std::string> &stop)
{
  std::set<std::string> next; ControlStrategy mode;
  if (!nextCommands(start,stop,next,mode)) return return_type::ERROR;
  if (next == active_commands_) return return_type::OK;
  resetCommands();
  if (!doSwitch(mode)) return return_type::ERROR;
  active_commands_=std::move(next); return return_type::OK;
}
bool LWRHW::doSwitch(ControlStrategy strategy) {current_strategy_=strategy; return true;}

bool LWRHW::enforceLimits(const rclcpp::Duration &period)
{
  if (period.nanoseconds() <= 0) return false;
  for (size_t j = 0; j < 7; ++j) {
    const auto &limit=limits_[j]; const auto &soft=soft_limits_[j];
    if (!std::isfinite(joint_position_command_[j]) || !std::isfinite(joint_set_point_command_[j]) ||
        !std::isfinite(joint_effort_command_[j]) || !std::isfinite(joint_stiffness_command_[j]) ||
        !std::isfinite(joint_damping_command_[j])) return false;
    double velocity_min=-limit.max_velocity, velocity_max=limit.max_velocity;
    if (has_soft_limits_[j]) {
      velocity_min=std::clamp(-soft.k_position*(joint_position_[j]-soft.min_position), -limit.max_velocity, limit.max_velocity);
      velocity_max=std::clamp(-soft.k_position*(joint_position_[j]-soft.max_position), -limit.max_velocity, limit.max_velocity);
    }
    const double lower=std::max(limit.min_position, previous_position_command_[j]+velocity_min*period.seconds());
    const double upper=std::min(limit.max_position, previous_position_command_[j]+velocity_max*period.seconds());
    if (lower > upper) {joint_position_command_[j]=std::clamp(joint_position_[j],limit.min_position,limit.max_position);}
    else joint_position_command_[j]=std::clamp(joint_position_command_[j],lower,upper);
    previous_position_command_[j]=joint_position_command_[j];
    joint_set_point_command_[j]=std::clamp(joint_set_point_command_[j],limit.min_position,limit.max_position);
    double effort_min=-limit.max_effort, effort_max=limit.max_effort;
    if (has_soft_limits_[j]) {
      effort_min=std::clamp(-soft.k_velocity*(joint_velocity_[j]-velocity_min), -limit.max_effort, limit.max_effort);
      effort_max=std::clamp(-soft.k_velocity*(joint_velocity_[j]-velocity_max), -limit.max_effort, limit.max_effort);
    }
    joint_effort_command_[j]=std::clamp(joint_effort_command_[j],effort_min,effort_max);
    joint_stiffness_command_[j]=std::clamp(joint_stiffness_command_[j],joint_lower_limits_stiffness_[j],joint_upper_limits_stiffness_[j]);
    joint_damping_command_[j]=std::clamp(joint_damping_command_[j],joint_lower_limits_damping_[j],joint_upper_limits_damping_[j]);
  }
  for (const auto *values : {&cart_pos_command_, &cart_stiff_command_, &cart_damp_command_, &cart_wrench_command_})
    for (double value : *values) if (!std::isfinite(value)) return false;
  return true;
}
}
