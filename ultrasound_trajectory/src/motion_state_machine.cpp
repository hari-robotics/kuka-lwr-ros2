#include "ultrasound_trajectory/motion_state_machine.h"

#include <algorithm>
#include <cmath>

namespace trm {

MotionStateMachine::MotionStateMachine(JointStateMapper& joint_mapper, IkSafetyGate& ik_gate,
                                        LegacyPosePublisher& pose_pub, Diagnostics& diagnostics,
                                        const TrmLimits& limits)
    : joint_mapper_(joint_mapper),
      ik_gate_(ik_gate),
      pose_pub_(pose_pub),
      diagnostics_(diagnostics),
      limits_(limits) {}

bool MotionStateMachine::submit(const MoveRequest& req) {
    if (state_ != MotionState::IDLE) return false;
    pending_request_ = req;
    have_pending_request_ = true;
    return true;
}

void MotionStateMachine::cancel() { cancel_requested_ = true; }

void MotionStateMachine::reset() { reset_requested_ = true; }

void MotionStateMachine::handleResetRequest(const rclcpp::Time& now) {
    if (!reset_requested_) return;
    reset_requested_ = false;

    if (state_ == MotionState::HOLD) {
        state_ = MotionState::IDLE;
        fault_reason_.clear();
        ik_gate_.resetContinuity();
    } else if (state_ == MotionState::FAULT) {
        // Guide 3.2 FAULT row: "operator reset -> IDLE". Re-check the
        // representative safety-critical condition (fresh joint
        // feedback) rather than resetting unconditionally.
        KDL::JntArray q;
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        const bool feedback_ok = joint_mapper_.snapshot(q, stamp) &&
                                  (now - stamp).seconds() <= limits_.feedback_timeout;
        if (feedback_ok) {
            state_ = MotionState::IDLE;
            fault_reason_.clear();
            ik_gate_.resetContinuity();
        }
        // else: remain latched in FAULT; fault_reason_ is left as-is.
    }
}

void MotionStateMachine::handleCancelRequest(const rclcpp::Time& now) {
    if (!cancel_requested_) return;
    cancel_requested_ = false;

    if (state_ == MotionState::PLANNING || state_ == MotionState::EXECUTING ||
        state_ == MotionState::SETTLING) {
        enterHold(now);
    }
}

void MotionStateMachine::tick(const rclcpp::Time& now) {
    double dt = 0.0;
    if (have_last_tick_time_) {
        dt = (now - last_tick_time_).seconds();
    }
    last_tick_time_ = now;
    have_last_tick_time_ = true;

    handleResetRequest(now);
    handleCancelRequest(now);

    switch (state_) {
        case MotionState::IDLE:
            if (have_pending_request_) {
                have_pending_request_ = false;
                enterPlanning(now);
            }
            break;
        case MotionState::EXECUTING:
            stepExecuting(now, dt);
            break;
        case MotionState::SETTLING:
            stepSettling(now, dt);
            break;
        case MotionState::PLANNING:
        case MotionState::HOLD:
        case MotionState::FAULT:
        default:
            break;  // no-op per guide 3.2 ("no automatic resume")
    }

    publishDiagnostics(now);
}

void MotionStateMachine::enterPlanning(const rclcpp::Time& now) {
    state_ = MotionState::PLANNING;

    KDL::JntArray q;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    if (!joint_mapper_.snapshot(q, stamp)) {
        fault("no_joint_feedback: never received a valid /joint_states sample", now);
        return;
    }
    const double age = (now - stamp).seconds();
    if (age > limits_.feedback_timeout) {
        fault("no_joint_feedback: stale (" + std::to_string(age) + "s)", now);
        return;
    }

    const KDL::Frame T_start = ik_gate_.fk(q);

    std::string reject;
    if (!traj_.plan(T_start, pending_request_, limits_, reject)) {
        fault("invalid_request: " + reject, now);
        return;
    }

    // Guide 4.3: "lengthen T if the sampled trajectory violates ...
    // predicted joint velocity, or predicted joint acceleration."
    // Se3Trajectory::plan() only sized T against the Cartesian limits
    // (it has no Jacobian); check the joint-space limits here, once,
    // using a whole-move joint-delta estimate, so the reactive per-tick
    // gate in stepExecuting() isn't left to discover a too-short
    // duration on its own and spend its stretchDuration() budget
    // compensating for it mid-trajectory.
    const Eigen::VectorXd dq_whole = ik_gate_.estimateDeltaQ(q, T_start, traj_.goal());
    if (dq_whole.size() > 0) {
        const double dq_total = dq_whole.cwiseAbs().maxCoeff();
        double t_needed = 0.0;
        if (limits_.qdot_pred_max > 0.0) {
            t_needed = std::max(t_needed,
                                 Se3Trajectory::kQuinticPeakVelFactor * dq_total / limits_.qdot_pred_max);
        }
        if (limits_.qddot_pred_max > 0.0 && dq_total > 0.0) {
            t_needed = std::max(t_needed,
                                 std::sqrt(Se3Trajectory::kQuinticPeakAccelFactor * dq_total /
                                           limits_.qddot_pred_max));
        }
        traj_.ensureMinDuration(t_needed);
    }

    u_current_ = 0.0;
    ik_gate_.resetContinuity();
    last_published_waypoint_ = T_start;
    stalling_ = false;
    state_ = MotionState::EXECUTING;
}

void MotionStateMachine::stepExecuting(const rclcpp::Time& now, double dt) {
    KDL::JntArray q;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    if (!joint_mapper_.snapshot(q, stamp)) {
        fault("no_joint_feedback: never received a valid /joint_states sample", now);
        return;
    }
    const double age = (now - stamp).seconds();
    if (age > limits_.feedback_timeout) {
        fault("no_joint_feedback: stale (" + std::to_string(age) + "s)", now);
        return;
    }
    const KDL::Frame T_actual = ik_gate_.fk(q);

    // Guide 8.3: following-error gate before advancing the trajectory
    // clock.
    const KDL::Twist err = KDL::diff(T_actual, last_published_waypoint_);
    last_pos_err_ = err.vel.Norm();
    last_rot_err_ = err.rot.Norm();

    if (last_pos_err_ > limits_.following_error_abort_position ||
        last_rot_err_ > limits_.following_error_abort_orientation) {
        fault("following_error_abort: instantaneous limit exceeded", now);
        return;
    }

    if (last_pos_err_ > limits_.following_position_gate ||
        last_rot_err_ > limits_.following_orientation_gate) {
        if (!stalling_) {
            stalling_ = true;
            stall_start_time_ = now;
        } else if ((now - stall_start_time_).seconds() > limits_.following_error_abort_duration) {
            fault("following_error_abort: gate exceeded too long", now);
        }
        return;  // do not advance the trajectory clock
    }
    stalling_ = false;

    double T = traj_.duration();
    if (T <= 0.0) T = limits_.t_min;
    const double du_nominal = (dt > 0.0) ? (dt / T) : 0.0;
    const double eval_dt = (dt > 1e-6) ? dt : (1.0 / std::max(limits_.publish_rate, 1.0));

    double du_try = du_nominal;
    bool accepted = false;
    GateResult gate_result;
    KDL::Frame candidate;
    double u_candidate = u_current_;

    // Guide 5.2: shrink the scalar increment and retry rather than
    // silently clipping a joint; never regress below u_current_.
    for (int attempt = 0; attempt < 8; ++attempt) {
        u_candidate = std::min(u_current_ + du_try, 1.0);
        const double s = Se3Trajectory::quinticScale(u_candidate);
        candidate = traj_.sampleAtScale(s);
        gate_result = ik_gate_.evaluate(q, T_actual, candidate, eval_dt);
        if (gate_result.accepted) {
            accepted = true;
            break;
        }
        if (du_try <= limits_.ds_min) break;
        du_try = std::max(du_try * 0.5, 0.0);
    }

    last_gate_result_ = gate_result;

    if (!accepted) {
        // Guide 5.2.4: lengthen the trajectory rather than clip.
        if (traj_.stretchDuration(2.0)) {
            // The trajectory's timing law just changed, so the qdot/qddot
            // baseline recorded under the old (faster) duration is no
            // longer a meaningful comparison point -- comparing against it
            // would make the *next* candidate's acceleration estimate
            // worse, not better, as duration keeps growing (its velocity
            // keeps shrinking toward zero while the stale baseline does
            // not). Reset so the next accepted step re-baselines under the
            // new duration instead. Safe: this only clears qdot/qddot/
            // branch-jump history, not dq_waypoint_max or joint-limit
            // margin checks, which stay fully enforced.
            ik_gate_.resetContinuity();
            return;  // try again next tick at the now-slower nominal rate
        }
        fault("ik_gate rejected: " + gate_result.reject_reason, now);
        return;
    }

    u_current_ = u_candidate;
    pose_pub_.publishFullPose(candidate);
    last_published_waypoint_ = candidate;

    if (u_current_ >= 1.0 - 1e-9) {
        state_ = MotionState::SETTLING;
        settle_stable_started_ = false;
        settle_enter_time_ = now;
        have_prev_settle_q_ = false;
    }
}

void MotionStateMachine::stepSettling(const rclcpp::Time& now, double /*dt*/) {
    KDL::JntArray q;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    if (!joint_mapper_.snapshot(q, stamp)) {
        fault("no_joint_feedback: never received a valid /joint_states sample", now);
        return;
    }
    const double age = (now - stamp).seconds();
    if (age > limits_.feedback_timeout) {
        fault("no_joint_feedback: stale (" + std::to_string(age) + "s)", now);
        return;
    }
    const KDL::Frame T_actual = ik_gate_.fk(q);

    const KDL::Twist err = KDL::diff(T_actual, traj_.goal());
    last_pos_err_ = err.vel.Norm();
    last_rot_err_ = err.rot.Norm();

    double max_qdot = 0.0;
    if (have_prev_settle_q_) {
        const double dt_s = (now - prev_settle_stamp_).seconds();
        if (dt_s > 1e-6) {
            for (unsigned int i = 0; i < q.rows(); ++i) {
                max_qdot = std::max(max_qdot, std::fabs(q(i) - prev_settle_q_(i)) / dt_s);
            }
        }
    }
    prev_settle_q_ = q;
    prev_settle_stamp_ = now;
    have_prev_settle_q_ = true;

    const bool within = (last_pos_err_ < limits_.settle_position_threshold) &&
                         (last_rot_err_ < limits_.settle_orientation_threshold) &&
                         (max_qdot < limits_.settle_joint_speed_threshold);

    if (within) {
        if (!settle_stable_started_) {
            settle_stable_started_ = true;
            settle_stable_since_ = now;
        } else if ((now - settle_stable_since_).seconds() >= limits_.settle_hold_duration) {
            state_ = MotionState::IDLE;
            have_prev_settle_q_ = false;
            return;
        }
    } else {
        settle_stable_started_ = false;
    }

    if ((now - settle_enter_time_).seconds() > limits_.settle_timeout) {
        fault("settle_timeout", now);
    }
}

void MotionStateMachine::enterHold(const rclcpp::Time& now) {
    state_ = MotionState::HOLD;

    KDL::JntArray q;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    if (joint_mapper_.snapshot(q, stamp) && (now - stamp).seconds() <= limits_.feedback_timeout) {
        const KDL::Frame T_actual = ik_gate_.fk(q);
        pose_pub_.publishFullPose(T_actual);
        last_published_waypoint_ = T_actual;
    }
    // else: best-effort hold only -- no fresh feedback to hold at, so
    // nothing further is published. The legacy controller was already
    // holding its own last received target.
}

void MotionStateMachine::fault(const std::string& reason, const rclcpp::Time& now) {
    if (state_ == MotionState::FAULT) return;  // already latched

    fault_reason_ = reason;
    state_ = MotionState::FAULT;

    KDL::JntArray q;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    if (joint_mapper_.snapshot(q, stamp) && (now - stamp).seconds() <= limits_.feedback_timeout) {
        const KDL::Frame T_actual = ik_gate_.fk(q);
        pose_pub_.publishFullPose(T_actual);
        last_published_waypoint_ = T_actual;
    }
}

void MotionStateMachine::publishDiagnostics(const rclcpp::Time& now) {
    DiagMetrics m;
    m.state = state_;
    m.fault_reason = fault_reason_.empty() ? "none" : fault_reason_;

    KDL::JntArray q;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    if (joint_mapper_.snapshot(q, stamp)) {
        m.joint_state_age = (now - stamp).seconds();
    } else {
        m.joint_state_age = -1.0;
    }

    m.position_error = last_pos_err_;
    m.orientation_error = last_rot_err_;
    m.max_predicted_joint_step =
        last_gate_result_.delta_q.size() > 0 ? last_gate_result_.delta_q.cwiseAbs().maxCoeff() : 0.0;
    m.min_singular_value = last_gate_result_.sigma_min;
    m.damping_lambda = last_gate_result_.lambda;
    m.min_joint_limit_margin = last_gate_result_.min_joint_margin;
    m.progress_s = Se3Trajectory::quinticScale(u_current_);

    diagnostics_.update(m);
}

}  // namespace trm
