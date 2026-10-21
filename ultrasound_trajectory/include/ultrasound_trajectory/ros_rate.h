// ROS 2 port of ros::Rate (roscpp rate.cpp): same cycle logic, on the node's ROS clock.
// rclcpp::Rate always uses the system clock; ros::Rate followed ROS time (/clock under
// use_sim_time). The /clock subscription runs on the node's clock thread
// (NodeOptions::use_clock_thread, default true), so sleep() also advances when called from a
// subscription callback of a single-threaded executor, as in ROS 1.
#ifndef ULTRASOUND_TRAJECTORY_ROS_RATE_H
#define ULTRASOUND_TRAJECTORY_ROS_RATE_H

#include "rclcpp/rclcpp.hpp"

namespace ultrasound_trajectory
{

class RosRate
{
public:
  RosRate(const rclcpp::Node::SharedPtr &node, double frequency)
  : clock_(node->get_clock()), start_(clock_->now()),
    expected_cycle_time_(rclcpp::Duration::from_seconds(1.0 / frequency)),
    actual_cycle_time_(0, 0)
  {}

  bool sleep()
  {
    rclcpp::Time expected_end = start_ + expected_cycle_time_;
    rclcpp::Time actual_end = clock_->now();

    // detect backward jumps in time
    if (actual_end < start_)
    {
      expected_end = actual_end + expected_cycle_time_;
    }

    // calculate the time we'll sleep for
    rclcpp::Duration sleep_time = expected_end - actual_end;

    // set the actual amount of time the loop took in case the user wants to know
    actual_cycle_time_ = actual_end - start_;

    // make sure to reset our start time
    start_ = expected_end;

    // if we've taken too much time we won't sleep
    if (sleep_time <= rclcpp::Duration(0, 0))
    {
      // if we've jumped forward in time, or the loop has taken more than a full extra
      // cycle, reset our cycle
      if (actual_end > expected_end + expected_cycle_time_)
      {
        start_ = actual_end;
      }
      // return false to show that the desired rate was not met
      return false;
    }

    return clock_->sleep_until(expected_end);
  }

  void reset() { start_ = clock_->now(); }

  rclcpp::Duration cycleTime() const { return actual_cycle_time_; }

  rclcpp::Duration expectedCycleTime() const { return expected_cycle_time_; }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time start_;
  rclcpp::Duration expected_cycle_time_;
  rclcpp::Duration actual_cycle_time_;
};

}  // namespace ultrasound_trajectory

#endif  // ULTRASOUND_TRAJECTORY_ROS_RATE_H
