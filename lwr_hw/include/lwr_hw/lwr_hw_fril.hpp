#ifndef LWR_HW__LWR_HW_FRIL_H
#define LWR_HW__LWR_HW_FRIL_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"

// FRIL remote hooks
#include <FastResearchInterface.h>
#include <control_toolbox/filters.hpp>
#include <boost/shared_ptr.hpp>
#include <filesystem>
#include <iostream>

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK   2000
#define EOK 0

namespace lwr_hw
{

class LWRHWFRIL : public LWRHW
{

public:

  LWRHWFRIL() : LWRHW() {}
  ~LWRHWFRIL() {}

  bool stop() override {return !device_ || device_->StopRobot() == EOK;};
  void set_mode(){return;};

  void setInitFile(std::string init_file){init_file_ = init_file; file_set_ = true;};

  // Init, read, and write, with FRI hooks
  bool init() override
  {
    if( !(file_set_) )
    {
      std::cout << "Did you forget to set the init file?" << std::endl
                << "You must do that before init()" << std::endl
                << "Exiting..." << std::endl;
      return false;
    }

    // construct a low-level lwr
    device_.reset( new FastResearchInterface( init_file_.c_str() ) );

    ResultValue	=	device_->StartRobot( FRI_CONTROL_POSITION );
    if (ResultValue != EOK)
    {
      std::cout << "An error occurred during starting up the robot...\n" << std::endl;
      return false;
    }

    float measured[7];
    device_->GetMeasuredJointPositions(measured);
    for (int j=0; j<7; ++j) {joint_position_[j]=measured[j];joint_position_prev_[j]=measured[j];}
    return true;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &period) override
  {
    if (!active_ || period.nanoseconds() <= 0) return hardware_interface::return_type::OK;
    float msrJntPos[n_joints_];
    float msrJntTrq[n_joints_];

    device_->GetMeasuredJointPositions( msrJntPos );
    device_->GetMeasuredJointTorques( msrJntTrq );

    for (int j = 0; j < n_joints_; j++)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] = (double)msrJntPos[j];
      joint_position_kdl_(j) = joint_position_[j];
      joint_effort_[j] = (double)msrJntTrq[j];
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.seconds(), joint_velocity_[j], 0.2);
      joint_stiffness_[j] = joint_stiffness_command_[j];
      joint_damping_[j] = joint_damping_command_[j];
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &period) override
  {
    if (!active_ || period.nanoseconds() <= 0) return hardware_interface::return_type::OK;
    pollEmergencyStop();
    if (!enforceLimits(period)) return hardware_interface::return_type::ERROR;

    // ensure the robot is powered and it is in control mode, almost like the isMachineOk() of Standford
    if ( device_->IsMachineOK() )
    {
      device_->WaitForKRCTick();

      switch (getControlStrategy())
      {

        case JOINT_POSITION:

          // Ensure the robot is in this mode
          if( (device_->GetCurrentControlScheme() == FRI_CONTROL_POSITION) )
          {
             float newJntPosition[n_joints_];
             for (int j = 0; j < n_joints_; j++)
             {
               newJntPosition[j] = (float)joint_position_command_[j];
             }
             device_->SetCommandedJointPositions(newJntPosition);
          }
          break;

        case CARTESIAN_IMPEDANCE:
          break;

         case JOINT_EFFORT:
         case JOINT_IMPEDANCE:

          // Ensure the robot is in this mode
          if( (device_->GetCurrentControlScheme() == FRI_CONTROL_JNT_IMP) )
          {
           float newJntPosition[n_joints_];
           float newJntStiff[n_joints_];
           float newJntDamp[n_joints_];
           float newJntAddTorque[n_joints_];

           // WHEN THE URDF MODEL IS PRECISE
           // 1. compute the gracity term
           // f_dyn_solver_->JntToGravity(joint_position_kdl_, gravity_effort_);

           // 2. read gravity term from FRI and add it with opposite sign and add the URDF gravity term
           // newJntAddTorque = gravity_effort_  - device_->getF_DYN??

            for(int j=0; j < n_joints_; j++)
            {
              newJntPosition[j] = (float)(getControlStrategy() == JOINT_EFFORT ? joint_position_[j] : joint_set_point_command_[j]);
              newJntAddTorque[j] = (float)joint_effort_command_[j];
              newJntStiff[j] = getControlStrategy() == JOINT_EFFORT ? 0.0f : (float)joint_stiffness_command_[j];
              newJntDamp[j] = getControlStrategy() == JOINT_EFFORT ? 0.0f : (float)joint_damping_command_[j];
            }
            device_->SetCommandedJointStiffness(newJntStiff);
            device_->SetCommandedJointPositions(newJntPosition);
            device_->SetCommandedJointDamping(newJntDamp);
            device_->SetCommandedJointTorques(newJntAddTorque);
          }
          break;

         case GRAVITY_COMPENSATION:
           break;
       }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
  {
    setInitFile(parameter("file"));
    std::error_code error;
    if (!std::filesystem::is_regular_file(init_file_, error)) {
      RCLCPP_ERROR(rclcpp::get_logger("lwr_hw"), "FRIL init file does not exist: %s", init_file_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return LWRHW::on_configure(state);
  }
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start,
      const std::vector<std::string> &stop) override
  {
    std::set<std::string> next; ControlStrategy mode;
    if (!nextCommands(start,stop,next,mode) || mode == CARTESIAN_IMPEDANCE)
      return hardware_interface::return_type::ERROR;
    return hardware_interface::return_type::OK;
  }
  bool doSwitch(ControlStrategy desired_strategy) override
  {
    if (desired_strategy == CARTESIAN_IMPEDANCE) return false; // original FRIL path did not implement it
    if (desired_strategy == current_strategy_) return true;
    if (!device_ || device_->StopRobot() != EOK) return false;
    const auto scheme = desired_strategy == JOINT_POSITION ? FRI_CONTROL_POSITION : FRI_CONTROL_JNT_IMP;
    ResultValue = device_->StartRobot(scheme);
    if (ResultValue != EOK) return false;
    setControlStrategy(desired_strategy); return true;
  }

private:

  // Parameters
  std::string init_file_;
  bool file_set_ = false;

  // low-level interface
  boost::shared_ptr<FastResearchInterface> device_;
  int ResultValue = 0;
};

}

#endif
