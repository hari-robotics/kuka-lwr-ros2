/*
 * motion_state_machine.h
 *
 * Guide Section 3.2 state machine (IDLE/PLANNING/EXECUTING/SETTLING/
 * HOLD/FAULT) and Section 8.3's feedback-paced publish loop. This is
 * the single owner of "when do we publish a new waypoint" -- guide
 * 8.4: "Use one owner thread for the motion state machine. Callbacks
 * only update request/feedback buffers." tick() is meant to be driven
 * by exactly one rclcpp::Timer on a single-threaded spinner (see
 * tool_relative_motion_node.cpp), which is what makes the lack of
 * internal locking in this class safe.
 */

#ifndef ULTRASOUND_TRAJECTORY_MOTION_STATE_MACHINE_H
#define ULTRASOUND_TRAJECTORY_MOTION_STATE_MACHINE_H

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "ultrasound_trajectory/diagnostics.h"
#include "ultrasound_trajectory/ik_safety_gate.h"
#include "ultrasound_trajectory/joint_state_mapper.h"
#include "ultrasound_trajectory/legacy_pose_publisher.h"
#include "ultrasound_trajectory/se3_trajectory.h"
#include "ultrasound_trajectory/trm_types.h"

namespace trm {

class MotionStateMachine {
public:
    MotionStateMachine(JointStateMapper& joint_mapper, IkSafetyGate& ik_gate,
                        LegacyPosePublisher& pose_pub, Diagnostics& diagnostics,
                        const TrmLimits& limits);

    // Guide table: only accepted from IDLE. A request arriving while
    // any other state is active is rejected outright, not queued (see
    // plan's flagged decision #2 -- the literal state table shows no
    // PLANNING/EXECUTING/SETTLING -> PLANNING transition).
    bool submit(const MoveRequest& req);

    // Accepted from PLANNING/EXECUTING/SETTLING -> HOLD.
    void cancel();

    // Accepted from HOLD always; from FAULT only if the underlying
    // condition (joint feedback freshness) is currently clear.
    void reset();

    // The sole place state transitions happen and waypoints are
    // published/evaluated. Call at publish_rate.
    void tick(const rclcpp::Time& now);

    MotionState state() const { return state_; }
    const std::string& faultReason() const { return fault_reason_; }

private:
    void handleResetRequest(const rclcpp::Time& now);
    void handleCancelRequest(const rclcpp::Time& now);

    void enterPlanning(const rclcpp::Time& now);
    void stepExecuting(const rclcpp::Time& now, double dt);
    void stepSettling(const rclcpp::Time& now, double dt);
    void enterHold(const rclcpp::Time& now);
    void fault(const std::string& reason, const rclcpp::Time& now);

    void publishDiagnostics(const rclcpp::Time& now);

    JointStateMapper& joint_mapper_;
    IkSafetyGate& ik_gate_;
    LegacyPosePublisher& pose_pub_;
    Diagnostics& diagnostics_;
    TrmLimits limits_;

    MotionState state_ = MotionState::IDLE;
    std::string fault_reason_;

    bool have_pending_request_ = false;
    MoveRequest pending_request_;

    bool cancel_requested_ = false;
    bool reset_requested_ = false;

    Se3Trajectory traj_;
    double u_current_ = 0.0;  // time-fraction progress in [0,1]; monotonic
                               // non-decreasing so a mid-trajectory
                               // stretchDuration() can never move it backward
    KDL::Frame last_published_waypoint_;

    bool have_last_tick_time_ = false;
    rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};

    bool stalling_ = false;
    rclcpp::Time stall_start_time_{0, 0, RCL_ROS_TIME};

    bool settle_stable_started_ = false;
    rclcpp::Time settle_stable_since_{0, 0, RCL_ROS_TIME};
    rclcpp::Time settle_enter_time_{0, 0, RCL_ROS_TIME};
    bool have_prev_settle_q_ = false;
    KDL::JntArray prev_settle_q_;
    rclcpp::Time prev_settle_stamp_{0, 0, RCL_ROS_TIME};

    // Latest-known values, kept only for diagnostics reporting.
    double last_pos_err_ = 0.0;
    double last_rot_err_ = 0.0;
    GateResult last_gate_result_;
};

}  // namespace trm

#endif  // ULTRASOUND_TRAJECTORY_MOTION_STATE_MACHINE_H
