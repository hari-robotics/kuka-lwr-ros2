#ifndef LWR_HW__LWR_HW_FRI_H
#define LWR_HW__LWR_HW_FRI_H
#include "lwr_hw/lwr_hw.h"
#include <control_toolbox/filters.hpp>
#include "fri/friudp.h"
#include "fri/friremote.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

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
    // As in ROS 1, a dedicated thread owns the friRemote transport and answers
    // every measurement packet at the rate set by the KRC (teach pendant/KRL),
    // independent of the ros2_control update rate. read/write only exchange
    // data with that thread, so a slower or jittery controller loop no longer
    // misses FRI replies, degrades quality and drops the robot out of command mode.
    stopCommunication();
    device_.reset();
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
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command_ = FriCommand{};
      msr_ = tFriMsrData{};
      packets_ = 0;
      applied_strategy_ = -1;
      next_diagnostics_ = std::chrono::steady_clock::time_point{};
    }
    communication_failed_ = false;
    stop_communication_ = false;
    communication_thread_ = std::thread(&LWRHWFRI::communicationLoop, this);
    // The initial command requests monitor mode (KRL int 1 = 0). A previous
    // quality fallback can leave KRL's command acknowledgement at one while FRI
    // is already in MONITOR, so synchronize a stop before starting again; an
    // initial KRL acknowledgement of 10 is already monitor-ready.
    tFriMsrData msr;
    if (!waitFor([](const tFriMsrData &) {return true;}, msr)) {stop(); return false;}
    if ((msr.intf.state != FRI_STATE_MON || msr.krl.intData[1] == 1) && !stopFRI()) {stop(); return false;}
    // KRL only skips strategy selection on its first start (acknowledgement 10).
    // Warm starts must explicitly select position strategy rather than send 0.
    if (!startFRI(JOINT_POSITION)) {stop(); return false;}
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msr = msr_;
      read_packets_ = packets_;
    }
    sampling_rate_ = msr.intf.desiredCmdSampleTime;
    if (!std::isfinite(sampling_rate_) || sampling_rate_ <= 0) {stop(); return false;}
    for (int j = 0; j < n_joints_; ++j) {
      joint_position_[j] = msr.data.msrJntPos[j];
      joint_position_prev_[j] = joint_position_[j];
      joint_position_kdl_(j) = joint_position_[j];
      joint_effort_[j] = msr.data.msrJntTrq[j];
      joint_velocity_[j] = 0.0;
    }
    for (int j = 0; j < 12; ++j) cart_pos_[j] = msr.data.msrCartPos[j];
    return true;
  }
  bool stop() override
  {
    if (!device_) return true;
    const bool stopped = communication_thread_.joinable() && stopFRI();
    stopCommunication();
    device_.reset();
    return stopped;
  }
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_) return hardware_interface::return_type::OK;
    if (communication_failed_) return hardware_interface::return_type::ERROR;
    tFriMsrData msr;
    unsigned long packets;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      packets = packets_;
      if (packets == read_packets_) return hardware_interface::return_type::OK;
      msr = msr_;
    }
    // Differentiate over the FRI packets actually received, not the controller period.
    const double dt = (packets - read_packets_) * static_cast<double>(sampling_rate_);
    read_packets_ = packets;
    for (int j = 0; j < n_joints_; j++)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] = msr.data.msrJntPos[j];
      joint_position_kdl_(j) = joint_position_[j];
      joint_effort_[j] = msr.data.msrJntTrq[j];
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/dt, joint_velocity_[j], 0.2);
      joint_stiffness_[j] = joint_stiffness_command_[j];
      joint_damping_[j] = joint_damping_command_[j];
    }
    for(int j = 0; j < 12; j++)
    {
        cart_pos_[j] = msr.data.msrCartPos[j];
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

    // Same commands as the ROS 1 write(); the communication thread sends them
    // with the reply to the next measurement packet.
    FriCommand command;
    command.krl_command = 1;
    command.strategy = getControlStrategy();
    command.software_hold = emergencyStopped();
    switch (getControlStrategy())
    {
      case JOINT_POSITION:
        command.type = FriCommand::POSITION;
        command.measured_position = false;
        copy(joint_position_command_, command.position);
        break;

      case CARTESIAN_IMPEDANCE:
        command.type = FriCommand::CARTESIAN_IMPEDANCE;
        command.measured_cartesian = false;
        command.has_cartesian_gains = true;
        copy(cart_pos_command_, command.cart_position);
        copy(cart_stiff_command_, command.cart_stiffness);
        copy(cart_damp_command_, command.cart_damping);
        copy(cart_wrench_command_, command.cart_wrench);
        break;

      case JOINT_IMPEDANCE:
        command.type = FriCommand::JOINT_IMPEDANCE;
        command.measured_position = false;
        command.has_stiffness = command.has_damping = command.has_torque = true;
        copy(joint_set_point_command_, command.position);
        copy(joint_stiffness_command_, command.stiffness);
        copy(joint_damping_command_, command.damping);
        copy(joint_effort_command_, command.torque);
        break;

     case JOINT_EFFORT:
        // mirror the position, zero stiffness, additional torque
        command.type = FriCommand::JOINT_IMPEDANCE;
        command.has_stiffness = command.has_torque = true;
        copy(joint_effort_command_, command.torque);
        break;

      case JOINT_STIFFNESS:
        command.type = FriCommand::JOINT_IMPEDANCE;
        command.measured_position = false;
        command.has_stiffness = true;
        copy(joint_set_point_command_, command.position);
        copy(joint_stiffness_command_, command.stiffness);
        break;

      case GRAVITY_COMPENSATION:
        command.type = FriCommand::JOINT_IMPEDANCE;
        break;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    // Never let a controller write override a pending KRL mode handshake.
    if (command_.krl_command == 1 && command_.strategy == command.strategy) command_ = command;
    return hardware_interface::return_type::OK;
  }

  bool doSwitch(ControlStrategy desired_strategy) override
  {
    if (communication_failed_) return false;
    if (desired_strategy == current_strategy_) return true;
    if (!device_ || !stopFRI()) return false;
    if (!startFRI(desired_strategy)) return false;
    setControlStrategy(desired_strategy); return true;
  }
  ~LWRHWFRI() override {stopCommunication();}
private:
  // Command handed from ros2_control to the communication thread. Positions
  // flagged as measured are taken from the packet being answered.
  struct FriCommand
  {
    enum Type {NONE, POSITION, JOINT_IMPEDANCE, CARTESIAN_IMPEDANCE} type = NONE;
    ControlStrategy strategy = JOINT_POSITION;
    int krl_command = 0;  // $FRI_FRM_INT[2]: 1 requests command mode, 0 monitor mode
    bool measured_position = true, has_stiffness = false, has_damping = false, has_torque = false;
    bool measured_cartesian = true, has_cartesian_gains = false;
    bool software_hold = false;  // emergency_stop hold, for diagnostics only
    float position[LBR_MNJ] = {}, stiffness[LBR_MNJ] = {}, damping[LBR_MNJ] = {}, torque[LBR_MNJ] = {};
    float cart_position[FRI_CART_FRM_DIM] = {}, cart_stiffness[FRI_CART_VEC] = {},
      cart_damping[FRI_CART_VEC] = {}, cart_wrench[FRI_CART_VEC] = {};
  };
  template<size_t N> static void copy(const std::vector<double> &from, float (&to)[N])
  {
    for (size_t i = 0; i < N && i < from.size(); ++i) to[i] = static_cast<float>(from[i]);
  }
  // Hold the measured pose while (re)entering command mode. KRL clears command
  // flags when a strategy changes; effort starts with zero stiffness and torque.
  static FriCommand hold(ControlStrategy strategy, int krl_command)
  {
    FriCommand command;
    command.strategy = strategy;
    command.krl_command = krl_command;
    if (strategy == JOINT_POSITION) command.type = FriCommand::POSITION;
    else if (strategy == CARTESIAN_IMPEDANCE) command.type = FriCommand::CARTESIAN_IMPEDANCE;
    else {
      command.type = FriCommand::JOINT_IMPEDANCE;
      command.has_torque = true;
      command.has_stiffness = strategy == JOINT_EFFORT;
    }
    return command;
  }

  int port_ = 49939;
  std::string hintToRemoteHost_ = "192.168.0.10";
  float sampling_rate_ = 0.0;
  int receive_timeout_ms_ = 100;
  std::unique_ptr<friRemote> device_;
  bool diagnostics_enabled_ = false;

  std::thread communication_thread_;
  std::atomic<bool> stop_communication_{false};
  std::atomic<bool> communication_failed_{false};
  std::mutex mutex_;
  std::condition_variable packet_received_;
  // Guarded by mutex_.
  FriCommand command_;
  tFriMsrData msr_{};
  unsigned long packets_ = 0;
  int applied_strategy_ = -1;
  std::chrono::steady_clock::time_point next_diagnostics_{};
  // Owned by the ros2_control thread.
  unsigned long read_packets_ = 0;

  void communicationLoop()
  {
    // Any failure (including a handshake timeout) stops the exchange until
    // reactivation, as before; the KRC then leaves command mode on its own.
    while (!stop_communication_ && !communication_failed_) {
      // Bounded by receive_timeout_ms; a lost connection disables the driver.
      if (device_->doReceiveData() != 0) {
        if (!stop_communication_)
          RCLCPP_ERROR(rclcpp::get_logger("lwr_hw"), "FRI packet receive failed; commands are disabled");
        break;
      }
      tFriCmdData transmitted;
      bool software_hold;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        msr_ = device_->getMsrBuf();
        ++packets_;
        apply(command_);
        transmitted = device_->getCmdBuf();
        software_hold = command_.software_hold;
      }
      packet_received_.notify_all();
      // Reply immediately after receiving, so the reflected sequence and the
      // latency belong to this packet.
      if (device_->doSendData() != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("lwr_hw"), "FRI packet send failed; commands are disabled");
        break;
      }
      if (diagnostics_enabled_) logDiagnostics(transmitted, software_hold);
    }
    if (!stop_communication_) communication_failed_ = true;
    packet_received_.notify_all();
  }
  // Called with mutex_ held, against the measurement just received.
  void apply(const FriCommand &command)
  {
    if (command.type != FriCommand::NONE && applied_strategy_ != command.strategy) {
      // KRL still understands 10/20/30, including zero-stiffness effort mode.
      device_->setToKRLInt(0, command.strategy == JOINT_EFFORT ? JOINT_IMPEDANCE : command.strategy);  // also clears the command flags
      applied_strategy_ = command.strategy;
    }
    device_->setToKRLInt(1, command.krl_command);
    float *measured = device_->getMsrMsrJntPosition();
    switch (command.type) {
      case FriCommand::NONE:
        break;
      case FriCommand::POSITION:
        device_->doPositionControl(command.measured_position ? measured :
          const_cast<float *>(command.position), false);
        break;
      case FriCommand::JOINT_IMPEDANCE:
        device_->doJntImpedanceControl(command.measured_position ? measured : command.position,
          command.has_stiffness ? command.stiffness : nullptr,
          command.has_damping ? command.damping : nullptr,
          command.has_torque ? command.torque : nullptr, false);
        break;
      case FriCommand::CARTESIAN_IMPEDANCE:
        device_->doCartesianImpedanceControl(
          command.measured_cartesian ? device_->getMsrCartPosition() : command.cart_position,
          command.has_cartesian_gains ? command.cart_stiffness : nullptr,
          command.has_cartesian_gains ? command.cart_damping : nullptr,
          command.has_cartesian_gains ? command.cart_wrench : nullptr, nullptr, false);
        break;
    }
  }
  void stopCommunication()
  {
    stop_communication_ = true;
    // The receive is bounded by receive_timeout_ms, so the join is too.
    if (communication_thread_.joinable()) communication_thread_.join();
  }
  // Waits for at least one new packet, then until done(msr) holds, with a 10 s deadline.
  template<typename Predicate> bool waitFor(Predicate done, tFriMsrData &msr)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    std::unique_lock<std::mutex> lock(mutex_);
    unsigned long seen = packets_;
    while (true) {
      if (!packet_received_.wait_until(lock, deadline, [&] {
            return packets_ != seen || communication_failed_ || stop_communication_;}))
        break;
      if (communication_failed_ || stop_communication_) return false;
      seen = packets_;
      if (done(msr_)) {msr = msr_; return true;}
    }
    RCLCPP_ERROR(rclcpp::get_logger("lwr_hw"), "Timed out waiting for the KRL FRI handshake");
    communication_failed_ = true;
    return false;
  }
  bool startFRI(ControlStrategy strategy)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command_ = hold(strategy, 1);
    }
    tFriMsrData msr;
    return waitFor([](const tFriMsrData &m) {
      return m.krl.intData[1] == 1 && m.intf.state == FRI_STATE_CMD;}, msr);
  }
  bool stopFRI()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command_.krl_command = 0;
    }
    tFriMsrData msr;
    return waitFor([](const tFriMsrData &m) {
      return m.krl.intData[1] == 0 && m.intf.state == FRI_STATE_MON;}, msr);
  }
  void logDiagnostics(const tFriCmdData &cmd, bool software_hold)
  {
    tFriMsrData msr;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (now < next_diagnostics_) return;
      next_diagnostics_ = now + std::chrono::seconds(1);
      msr = msr_;
    }
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
      msr.krl.intData[1], cmd.krl.intData[1], cmd.cmd.cmdFlags, software_hold,
      msr.data.msrJntPos[0], msr.data.msrJntPos[1], msr.data.msrJntPos[2],
      msr.data.msrJntPos[3], msr.data.msrJntPos[4], msr.data.msrJntPos[5], msr.data.msrJntPos[6],
      cmd.cmd.jntPos[0], cmd.cmd.jntPos[1], cmd.cmd.jntPos[2], cmd.cmd.jntPos[3],
      cmd.cmd.jntPos[4], cmd.cmd.jntPos[5], cmd.cmd.jntPos[6]);
  }
};
using LWRSystemHardware = LWRHWFRI;
}
#endif
