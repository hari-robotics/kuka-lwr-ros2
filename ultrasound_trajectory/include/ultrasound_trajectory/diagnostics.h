/*
 * diagnostics.h
 *
 * Guide 8.4: "Publish diagnostics: state, joint-state age,
 * position/orientation following error, maximum predicted joint step,
 * minimum singular value, joint-limit margin, and last fault." One
 * status line on a slower ~status topic, separate from the 100 Hz
 * planning tick, to keep bus load light.
 */

#ifndef ULTRASOUND_TRAJECTORY_DIAGNOSTICS_H
#define ULTRASOUND_TRAJECTORY_DIAGNOSTICS_H

#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ultrasound_trajectory/trm_types.h"

namespace trm {

struct DiagMetrics {
    MotionState state = MotionState::IDLE;
    std::string fault_reason = "none";
    double joint_state_age = 0.0;
    double position_error = 0.0;
    double orientation_error = 0.0;
    double max_predicted_joint_step = 0.0;
    double min_singular_value = 0.0;
    double damping_lambda = 0.0;
    double min_joint_limit_margin = 0.0;
    double progress_s = 0.0;
};

class Diagnostics {
public:
    explicit Diagnostics(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub) : pub_(pub) {}

    void update(const DiagMetrics& m) { metrics_ = m; }

    void publish() {
        std::ostringstream ss;
        ss << "state=" << toString(metrics_.state)
           << " fault=" << metrics_.fault_reason
           << " joint_age=" << metrics_.joint_state_age
           << " pos_err=" << metrics_.position_error
           << " rot_err=" << metrics_.orientation_error
           << " max_dq=" << metrics_.max_predicted_joint_step
           << " sigma_min=" << metrics_.min_singular_value
           << " lambda=" << metrics_.damping_lambda
           << " margin_min=" << metrics_.min_joint_limit_margin
           << " s=" << metrics_.progress_s;

        std_msgs::msg::String msg;
        msg.data = ss.str();
        pub_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    DiagMetrics metrics_;
};

}  // namespace trm

#endif  // ULTRASOUND_TRAJECTORY_DIAGNOSTICS_H
