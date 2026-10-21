#ifndef LWR_HW__LWR_HW_REAL_H
#define LWR_HW__LWR_HW_REAL_H

// lwr hw definition
#include "lwr_hw/lwr_hw.h"
#include <control_toolbox/filters.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>

// FRIL remote hooks
#include <FastResearchInterface.h>

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK   2000
#define EOK 0

namespace lwr_hw
{

class LWRHWFRIL : public LWRHW
{

public:

  LWRHWFRIL() : LWRHW() {}
  ~LWRHWFRIL() {}

  void stop() override {return;};
  void set_mode(){return;};

  void setInitFile(std::string init_file){init_file_ = init_file; file_set_ = true;};

  // lwr_hw_fril_node parameter: file (now a ros2_control hardware parameter)
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
  {
    setInitFile(parameter("file", ""));
    return LWRHW::on_configure(state);
  }

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

    return true;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &period) override
  {
    // ros2_control also reads an inactive (not yet started) hardware
    if (!device_) return hardware_interface::return_type::OK;

    float msrJntPos[n_joints_];
    float msrJntTrq[n_joints_];
    float msrExtJntTrq[n_joints_]; //ADDED 10.22

    device_->GetMeasuredJointPositions( msrJntPos );
    device_->GetMeasuredJointTorques( msrJntTrq );
    device_->GetEstimatedExternalJointTorques( msrExtJntTrq ); //ADDED 10.22

    for (int j = 0; j < n_joints_; j++)
    {
      joint_position_prev_[j] = joint_position_[j];
      joint_position_[j] = (double)msrJntPos[j];
      joint_position_kdl_(j) = joint_position_[j];
      joint_effort_[j] = (double)msrJntTrq[j];
      joint_est_ext_trq_[j] = (double)msrExtJntTrq[j]; //ADDED 10.22
      joint_velocity_[j] = filters::exponentialSmoothing((joint_position_[j]-joint_position_prev_[j])/period.seconds(), joint_velocity_[j], 0.2);
      joint_stiffness_[j] = joint_stiffness_command_[j];
      joint_damping_[j] = joint_damping_command_[j];
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &period) override
  {
    // ROS 1 lwr_hw_fril_node had no e-stop subscription (FRI node only)
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
              newJntPosition[j] = (float)joint_set_point_command_[j];
              newJntAddTorque[j] = (float)joint_effort_command_[j];
              newJntStiff[j] = (float)joint_stiffness_command_[j];
              newJntDamp[j] = (float)joint_damping_command_[j];
            }
            device_->SetCommandedJointStiffness(newJntStiff);
            device_->SetCommandedJointPositions(newJntPosition);
            device_->SetCommandedJointDamping(newJntDamp);
            device_->SetCommandedJointTorques(newJntAddTorque);
          }
          break;

         case GRAVITY_COMPENSATION:
           break;

         default:
           break;
       }
    }
    return hardware_interface::return_type::OK;
  }

  void doSwitch(ControlStrategy desired_strategy) override
  {

    ResultValue	=	device_->StopRobot();
    if (ResultValue != EOK)
    {
        std::cout << "An error occurred during stopping the robot, couldn't switch mode...\n" << std::endl;
        return;
    }

    // only allow joint position and joint impedance control strategies, otherwise set the default (JOINT_POSITION) strategy
    if(desired_strategy != JOINT_POSITION && desired_strategy != JOINT_IMPEDANCE)
        desired_strategy = JOINT_POSITION;

    semanticZero();

    if(desired_strategy == getControlStrategy())
    {
      std::cout << "The ControlStrategy didn't changed, it is already: " << getControlStrategy() << std::endl;
    }
    else
    {
      switch( desired_strategy )
      {
        case JOINT_POSITION:
          ResultValue = device_->StartRobot( FRI_CONTROL_POSITION );
          if (ResultValue != EOK)
          {
            std::cout << "An error occurred during starting the robot, couldn't switch to JOINT_POSITION...\n" << std::endl;
            return;
          }
          break;
         case JOINT_IMPEDANCE:
          ResultValue = device_->StartRobot( FRI_CONTROL_JNT_IMP );
          if (ResultValue != EOK)
          {
            std::cout << "An error occurred during starting the robot, couldn't switch to JOINT_IMPEDANCE...\n" << std::endl;
            return;
          }
          break;
         default:
          break;
      }

      // if sucess during the switch in FRI, set the ROS strategy
      setControlStrategy(desired_strategy);

      std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
    }
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
