#ifndef LWR_HW__LWR_HW_GAZEBO_H
#define LWR_HW__LWR_HW_GAZEBO_H
#include <gz_ros2_control/gz_system_interface.hpp>
#include "lwr_hw/lwr_hw.h"

namespace lwr_hw
{
// Gazebo Classic's ModelPlugin update loop is supplied by gz_ros2_control.
// The original joint order, filtering and effort + gravity law are retained.
class LWRHWGazebo : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  bool initSim(rclcpp::Node::SharedPtr &, std::map<std::string, sim::Entity> &,
    const hardware_interface::HardwareInfo &, sim::EntityComponentManager &, int &) override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &,
    const std::vector<std::string> &) override;
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &,
    const std::vector<std::string> &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  class Buffers : public LWRHW {
  public:
    bool init() override {return true;}
    bool stop() override {return true;}
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
      {return hardware_interface::return_type::OK;}
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
      {return hardware_interface::return_type::OK;}
  };
  Buffers buffers_;
  sim::EntityComponentManager *ecm_ = nullptr;
  std::vector<sim::Entity> sim_joints_;
  bool active_ = false;
};
}
#endif
