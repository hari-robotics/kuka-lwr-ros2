#ifndef LWR_HW__LWR_HW_FRI_H
#define LWR_HW__LWR_HW_FRI_H
#include "lwr_hw/lwr_hw.h"
#include <control_toolbox/filters.hpp>
#include "fri/friudp.h"
#include "fri/friremote.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>

namespace lwr_hw
{
class LWRHWFRI : public LWRHW
{
public:
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
  {
    // Opt-in host diagnostics only; does not alter FRI commands or mode selection.
    const char *diagnostics = std::getenv("LWR_FRI_DIAGNOSTICS");
    diagnostics_enabled_ = diagnostics && std::string(diagnostics) == "1";
    try {
      size_t used = 0;
      const auto text = parameter("port", "49939");
      port_ = std::stoi(text, &used);
      if (used != text.size() || port_ < 10 || port_ > 65535)
        throw std::runtime_error("Invalid FRI port");
      hintToRemoteHost_ = parameter("ip", "192.168.0.10");
      const auto timeout = parameter("receive_timeout_ms", "100");
      receive_timeout_ms_ = std::stoi(timeout, &used);
      if (used != timeout.size() || receive_timeout_ms_ < 1 || receive_timeout_ms_ > 10000)
        throw std::runtime_error("Invalid FRI receive_timeout_ms (expected 1..10000)");
      in_addr address{};
      if (inet_pton(AF_INET, hintToRemoteHost_.c_str(), &address) != 1)
        throw std::runtime_error("Invalid FRI port/ip");
    } catch (const std::exception &error) {
      RCLCPP_ERROR(rclcpp::get_logger("lwr_hw"), "%s", error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return LWRHW::on_configure(state);
  }
  void setPort(int port) {port_ = port;}
  void setIP(std::string ip) {hintToRemoteHost_ = std::move(ip);}
  float getSampleTime() const {return sampling_rate_;}
  bool init() override
  {
    // Keep the original friRemote transport and KRL handshake. ros2_control is
    // the sole owner of packet exchange instead of a racing background thread.
    device_.reset();
    communication_failed_ = false;
    next_diagnostics_ = std::chrono::steady_clock::time_point{};
    try {
      device_ = std::make_unique<friRemote>(port_, const_cast<char *>(hintToRemoteHost_.c_str()));
    } catch (const std::exception &error) {
      RCLCPP_ERROR(rclcpp::get_logger("lwr_hw"), "FRI initialization failed: %s", error.what());
      return false;
    }
    if (device_->setReceiveTimeout(receive_timeout_ms_) != 0) {
      device_.reset();
      return false;
    }
    // A previous quality fallback can leave KRL's command acknowledgement at
    // one while FRI is already in MONITOR. Synchronize a stop before starting
    // again; an initial KRL acknowledgement of 10 is already monitor-ready.
    device_->setToKRLInt(1, 0);
    if (!exchange()) {device_.reset();return false;}
    if ((device_->getState() != FRI_STATE_MON || device_->getFrmKRLInt(1) == 1) &&
        !stopFRI()) {device_.reset();return false;}
    // KRL only skips strategy selection on its first start (acknowledgement 10).
    // Warm starts must explicitly select position strategy rather than send 0.
    device_->setToKRLInt(0, JOINT_POSITION);
    if (!startFRI(JOINT_POSITION)) {device_.reset();return false;}
    sampling_rate_ = device_->getSampleTime();
    if (!std::isfinite(sampling_rate_) || sampling_rate_ <= 0) {device_.reset();return false;}
    for (int j = 0; j < n_joints_; ++j) {
      joint_position_[j] = device_->getMsrMsrJntPosition()[j];
      joint_position_prev_[j] = joint_position_[j];
      joint_position_kdl_(j) = joint_position_[j];
      joint_effort_[j] = device_->getMsrJntTrq()[j];
    }
    for (int j = 0; j < 12; ++j) cart_pos_[j] = device_->getMsrCartPosition()[j];
    return true;
  }
  bool stop() override
  {
    if (!device_) return true;
    const bool stopped = stopFRI();
    device_.reset();
    return stopped;
  }
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &period) override
  {
    if (!active_ || period.nanoseconds() <= 0) return hardware_interface::return_type::OK;
    if (!exchange()) return hardware_interface::return_type::ERROR;
    for (int j = 0; j < n_joints_; j++)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] = device_->getMsrMsrJntPosition()[j];
      joint_position_kdl_(j) = joint_position_[j];
      joint_effort_[j] = device_->getMsrJntTrq()[j];
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.seconds(), joint_velocity_[j], 0.2);
      joint_stiffness_[j] = joint_stiffness_command_[j];
      joint_damping_[j] = joint_damping_command_[j];
    }
    for(int j = 0; j < 12; j++)
    {
        cart_pos_[j] = device_->getMsrCartPosition()[j];
    }
    for(int j = 0; j < 6; j++)
    {
        cart_stiff_[j] = cart_stiff_command_[j];
        cart_damp_[j] = cart_damp_command_[j];
        cart_wrench_[j] = cart_wrench_command_[j];
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &period) override
  {
    if (communication_failed_) return hardware_interface::return_type::ERROR;
    if (!active_ || period.nanoseconds() <= 0) return hardware_interface::return_type::OK;
    pollEmergencyStop();
    if (!enforceLimits(period)) return hardware_interface::return_type::ERROR;

    float newJntPosition[n_joints_];
    float newJntStiff[n_joints_];
    float newJntDamp[n_joints_];
    float newJntAddTorque[n_joints_];
    float newCartPos[12];
    float newCartStiff[6];
    float newCartDamp[6];
    float newAddFT[6];

    switch (getControlStrategy())
    {
      case JOINT_POSITION:
        for (int j = 0; j < n_joints_; j++)
        {
          newJntPosition[j] = joint_position_command_[j];
        }
        device_->doPositionControl(newJntPosition, false);
        break;

      case CARTESIAN_IMPEDANCE:
        for(int i=0; i < 12; ++i)
        {
          newCartPos[i] = cart_pos_command_[i];
        }
        for(int i=0; i < 6; i++)
        {
          newCartStiff[i] = cart_stiff_command_[i];
          newCartDamp[i] = cart_damp_command_[i];
          newAddFT[i] = cart_wrench_command_[i];
        }
        device_->doCartesianImpedanceControl(newCartPos, newCartStiff, newCartDamp, newAddFT, NULL, false);
        break;

      case JOINT_IMPEDANCE:
        for(int j=0; j < n_joints_; j++)
        {
          newJntPosition[j] = joint_set_point_command_[j];
          newJntAddTorque[j] = joint_effort_command_[j];
          newJntStiff[j] = joint_stiffness_command_[j];
          newJntDamp[j] = joint_damping_command_[j];
        }
        device_->doJntImpedanceControl(newJntPosition, newJntStiff, newJntDamp, newJntAddTorque, false);
        break;

     case JOINT_EFFORT:
        for(int j=0; j < n_joints_; j++)
        {
            newJntAddTorque[j] = joint_effort_command_[j];
            newJntStiff[j] = 0.0;
        }
        // mirror the position
        device_->doJntImpedanceControl(device_->getMsrMsrJntPosition(), newJntStiff, NULL, newJntAddTorque, false);
        break;

      case JOINT_STIFFNESS:
        for(int j=0; j < n_joints_; j++)
        {
          newJntPosition[j] = joint_set_point_command_[j];
          newJntStiff[j] = joint_stiffness_command_[j];
        }
        device_->doJntImpedanceControl(newJntPosition, newJntStiff, NULL, NULL, false);
        break;

      case GRAVITY_COMPENSATION:
        device_->doJntImpedanceControl(device_->getMsrMsrJntPosition(), NULL, NULL, NULL, false);
        break;
    }
    return hardware_interface::return_type::OK;
  }

  bool doSwitch(ControlStrategy desired_strategy) override
  {
    if (communication_failed_) return false;
    if (desired_strategy == current_strategy_) return true;
    if (!device_ || !stopFRI()) return false;
    // KRL still understands 10/20/30, including zero-stiffness effort mode.
    const int wire_strategy = desired_strategy == JOINT_EFFORT ? JOINT_IMPEDANCE : desired_strategy;
    device_->setToKRLInt(0, wire_strategy);
    if (!startFRI(desired_strategy)) return false;
    setControlStrategy(desired_strategy); return true;
  }
private:
  int port_ = 49939;
  std::string hintToRemoteHost_ = "192.168.0.10";
  float sampling_rate_ = 0.0;
  int receive_timeout_ms_ = 100;
  bool communication_failed_ = false;
  std::unique_ptr<friRemote> device_;
  bool diagnostics_enabled_ = false;
  std::chrono::steady_clock::time_point next_diagnostics_{};
  tFriCmdData last_transmitted_command_{};
  bool exchange()
  {
    if (communication_failed_) return false;
    if (device_ && diagnostics_enabled_) last_transmitted_command_ = device_->getCmdBuf();
    // FRI sends measurements periodically. Reply immediately after receiving
    // the current measurement so the reflected sequence and latency belong to
    // this cycle. Sending before receive postpones the previous reply until
    // the next ros2_control read, adding a full update period to its latency.
    if (!device_ || device_->doReceiveData() != 0 || device_->doSendData() != 0) {
      communication_failed_ = true;
      RCLCPP_ERROR(rclcpp::get_logger("lwr_hw"), "FRI packet exchange failed; commands are disabled");
      return false;
    }
    if (diagnostics_enabled_) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_diagnostics_) {
        next_diagnostics_ = now + std::chrono::seconds(1);
        const auto &msr = device_->getMsrBuf();
        const auto &cmd = last_transmitted_command_;
        RCLCPP_INFO(rclcpp::get_logger("lwr_hw"),
          "FRI diagnostics: state=%u quality=%u power=0x%04x control=%u error=0x%04x warning=0x%04x "
          "sample_ms=%.3f answer_rate=%.3f latency_ms=%.3f jitter_ms=%.3f missed=%u "
          "krl_ack=%d krl_start_sent=%d cmd_flags=0x%04x software_hold=%d "
          "measured_rad=[%.6f %.6f %.6f %.6f %.6f %.6f %.6f] "
          "sent_rad=[%.6f %.6f %.6f %.6f %.6f %.6f %.6f]",
          msr.intf.state, msr.intf.quality, msr.robot.power, msr.robot.control,
          msr.robot.error, msr.robot.warning, msr.intf.desiredCmdSampleTime * 1000.0,
          msr.intf.stat.answerRate, msr.intf.stat.latency * 1000.0,
          msr.intf.stat.jitter * 1000.0, msr.intf.stat.missCounter,
          msr.krl.intData[1], cmd.krl.intData[1], cmd.cmd.cmdFlags, emergencyStopped(),
          msr.data.msrJntPos[0], msr.data.msrJntPos[1], msr.data.msrJntPos[2],
          msr.data.msrJntPos[3], msr.data.msrJntPos[4], msr.data.msrJntPos[5], msr.data.msrJntPos[6],
          cmd.cmd.jntPos[0], cmd.cmd.jntPos[1], cmd.cmd.jntPos[2], cmd.cmd.jntPos[3],
          cmd.cmd.jntPos[4], cmd.cmd.jntPos[5], cmd.cmd.jntPos[6]);
      }
    }
    return true;
  }
  bool startFRI(ControlStrategy strategy)
  {
    device_->setToKRLInt(1, 1);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    do {
      // Prepare a measured-pose hold before entering command mode. KRL clears
      // command flags when a strategy changes; effort starts with zero torque.
      float zero[LBR_MNJ] = {};
      if (strategy == JOINT_POSITION)
        device_->doPositionControl(device_->getMsrMsrJntPosition(), false);
      else if (strategy == CARTESIAN_IMPEDANCE)
        device_->doCartesianImpedanceControl(device_->getMsrCartPosition(), nullptr,
          nullptr, nullptr, nullptr, false);
      else
        device_->doJntImpedanceControl(device_->getMsrMsrJntPosition(),
          strategy == JOINT_EFFORT ? zero : nullptr, nullptr, zero, false);
      if (!exchange()) return false;
      if (std::chrono::steady_clock::now() > deadline) {
        communication_failed_ = true;
        return false;
      }
    } while (device_->getFrmKRLInt(1) != 1 || device_->getState() != FRI_STATE_CMD);
    return true;
  }
  bool stopFRI()
  {
    device_->setToKRLInt(1, 0);
    // Each receive is bounded, including a connection lost during switching.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    do {
      if (!exchange()) return false;
      if (std::chrono::steady_clock::now() > deadline) {communication_failed_ = true;return false;}
    } while (device_->getFrmKRLInt(1) != 0 || device_->getState() != FRI_STATE_MON);
    return true;
  }
};
using LWRSystemHardware = LWRHWFRI;
}
#endif
