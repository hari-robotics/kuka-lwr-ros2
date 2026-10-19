#ifndef KINEMATIC_CHAIN_CONTROLLER_BASE_H
#define KINEMATIC_CHAIN_CONTROLLER_BASE_H
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <lwr_controllers/ros2_adapter.h>
#include <vector>
#include <stdexcept>

namespace lwr_controllers {
struct EffortInterface {static constexpr const char *command = "effort";};
struct PositionInterface {static constexpr const char *command = "position";};
struct StateInterface {static constexpr const char *command = "";};
struct CartesianInterface {static constexpr const char *command = "cartesian";};
// Small handle adapter: every active handle points at ros2_control's loaned interfaces.
struct JointHandle {
  std::string name;
  hardware_interface::LoanedStateInterface *position=nullptr,*velocity=nullptr,*effort=nullptr;
  hardware_interface::LoanedCommandInterface *command=nullptr;
  const std::string &getName() const {return name;}
  double getPosition() const {return position ? position->get_value() : 0.0;}
  double getVelocity() const {return velocity ? velocity->get_value() : 0.0;}
  double getEffort() const {return effort ? effort->get_value() : 0.0;}
  void setCommand(double value) {if(!command) throw std::runtime_error("Unbound command interface: "+name);command->set_value(value);}
};
template<typename Mode> class ControllerAdapter: public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override {return controller_interface::CallbackReturn::SUCCESS;}
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    nh_=NodeParameters(get_node(),&callback_mutex_);
    joint_names_.clear();joint_handles_.clear();
    try {return configure(nh_) ? controller_interface::CallbackReturn::SUCCESS : controller_interface::CallbackReturn::ERROR;}
    catch(const std::exception &e){RCLCPP_ERROR(get_node()->get_logger(),"Configuration failed: %s",e.what());return controller_interface::CallbackReturn::ERROR;}
  }
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration config{controller_interface::interface_configuration_type::INDIVIDUAL,{}};
    if constexpr(!std::is_same_v<Mode,StateInterface>)
      for(const auto &name:joint_names_) config.names.push_back(name+"/"+Mode::command);
    return config;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    controller_interface::InterfaceConfiguration config{controller_interface::interface_configuration_type::INDIVIDUAL,{}};
    for(const auto &name:joint_names_)
      for(const auto &interface:{"position","velocity","effort"}) config.names.push_back(name+"/"+interface);
    return config;
  }
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    try {
      for(auto &handle:joint_handles_) {
        handle.position=find_state(handle.name+"/position");
        handle.velocity=find_state(handle.name+"/velocity");
        handle.effort=find_state(handle.name+"/effort");
        if constexpr(!std::is_same_v<Mode,StateInterface>) handle.command=find_command(handle.name+"/"+Mode::command);
      }
      starting(get_node()->now());
      return controller_interface::CallbackReturn::SUCCESS;
    }catch(const std::exception &e){RCLCPP_ERROR(get_node()->get_logger(),"Activation failed: %s",e.what());return controller_interface::CallbackReturn::ERROR;}
  }
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    stopping(get_node()->now());
    for(auto &handle:joint_handles_) {
      if(handle.command && std::string(Mode::command)=="effort") handle.command->set_value(0.0);
      handle.position=nullptr;handle.velocity=nullptr;handle.effort=nullptr;handle.command=nullptr;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::return_type update(const rclcpp::Time &time,const rclcpp::Duration &period) override {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if(period.nanoseconds()<=0) return controller_interface::return_type::OK;
    try {update_command(time,period);return controller_interface::return_type::OK;}
    catch(const std::exception &e){RCLCPP_ERROR(get_node()->get_logger(),"Update failed: %s",e.what());return controller_interface::return_type::ERROR;}
  }
  virtual bool configure(NodeParameters &node)=0;
  virtual void starting(const rclcpp::Time &) {}
  virtual void stopping(const rclcpp::Time &) {}
  virtual void update_command(const rclcpp::Time &,const rclcpp::Duration &)=0;
protected:
  NodeParameters nh_;
  std::mutex callback_mutex_;
  std::vector<std::string> joint_names_;
  std::vector<JointHandle> joint_handles_;
  hardware_interface::LoanedStateInterface *find_state(const std::string &name) {
    for(auto &interface:state_interfaces_) if(interface.get_name()==name) return &interface;
    throw std::runtime_error("Missing state interface: "+name);
  }
  hardware_interface::LoanedCommandInterface *find_command(const std::string &name) {
    for(auto &interface:command_interfaces_) if(interface.get_name()==name) return &interface;
    throw std::runtime_error("Missing command interface: "+name);
  }
};
}

namespace controller_interface {
template<typename Mode> class KinematicChainControllerBase: public lwr_controllers::ControllerAdapter<Mode> {
public:
  bool configure(lwr_controllers::NodeParameters &node) override {
    this->nh_=node;
    std::string xml,root_name,tip_name;
    if(!node.getParam("robot_description",xml) || !node.getParam("root_name",root_name) || !node.getParam("tip_name",tip_name)) {
      RCLCPP_ERROR(node.node()->get_logger(),"robot_description, root_name and tip_name must be supplied to this controller");return false;
    }
    urdf::Model model;KDL::Tree tree;
    if(!model.initString(xml) || !kdl_parser::treeFromUrdfModel(model,tree) || !tree.getChain(root_name,tip_name,kdl_chain_)) return false;
    gravity_=KDL::Vector(0,0,-9.81);
    auto count=kdl_chain_.getNrOfJoints();
    if(count==0) return false;
    joint_limits_.min.resize(count);joint_limits_.max.resize(count);joint_limits_.center.resize(count);
    this->joint_names_.clear();this->joint_handles_.clear();
    size_t index=0;
    for(const auto &segment:kdl_chain_.segments) if(segment.getJoint().getType()!=KDL::Joint::None) {
      auto name=segment.getJoint().getName();auto joint=model.getJoint(name);
      if(!joint || !joint->limits) return false;
      joint_limits_.min(index)=joint->limits->lower;joint_limits_.max(index)=joint->limits->upper;
      joint_limits_.center(index)=(joint->limits->lower+joint->limits->upper)/2.0;
      this->joint_names_.push_back(name);this->joint_handles_.push_back(lwr_controllers::JointHandle{name});++index;
    }
    joint_msr_states_.resize(count);joint_des_states_.resize(count);
    return true;
  }
protected:
  using lwr_controllers::ControllerAdapter<Mode>::nh_;
  using lwr_controllers::ControllerAdapter<Mode>::joint_handles_;
  KDL::Chain kdl_chain_;
  KDL::Vector gravity_;
  KDL::JntArrayAcc joint_msr_states_,joint_des_states_;
  struct Limits {KDL::JntArray min,max,center;} joint_limits_;
};
}
#endif
