/*
 * trm_types.h
 *
 * Shared, dependency-light types for the tool_relative_motion_node.
 * No ROS/topic dependency -- only Eigen/KDL/std types -- so the
 * trajectory and IK-gate math can be reasoned about (and, later,
 * unit-tested) independently of the ROS graph.
 *
 * See: KUKA_LWR4_Tool_Relative_Motion_Development_Guide.md, Sections 3.2,
 * 4, 5, 6, Appendix A.
 */

#ifndef ULTRASOUND_TRAJECTORY_TRM_TYPES_H
#define ULTRASOUND_TRAJECTORY_TRM_TYPES_H

#include <string>
#include <Eigen/Core>

namespace trm {

// Guide Section 3.2 state table.
enum class MotionState {
    IDLE,
    PLANNING,
    EXECUTING,
    SETTLING,
    HOLD,
    FAULT
};

inline const char* toString(MotionState s) {
    switch (s) {
        case MotionState::IDLE:      return "IDLE";
        case MotionState::PLANNING:  return "PLANNING";
        case MotionState::EXECUTING: return "EXECUTING";
        case MotionState::SETTLING:  return "SETTLING";
        case MotionState::HOLD:      return "HOLD";
        case MotionState::FAULT:     return "FAULT";
    }
    return "UNKNOWN";
}

// A tool-frame delta request, guide Section 1.1/7.1 (Twist-based, see
// ~move topic: linear = dx,dy,dz [m], angular = droll,dpitch,dyaw [rad]).
struct MoveRequest {
    double dx = 0.0, dy = 0.0, dz = 0.0;
    double droll = 0.0, dpitch = 0.0, dyaw = 0.0;
    double requested_duration = 0.0;  // <=0 means "let the planner choose"
};

// Guide Section 6 safety envelope + Section 7.2 parameters, all in SI
// units (metres, radians, seconds).
struct TrmLimits {
    double request_translation_max = 0.030;
    double request_rotation_max    = 0.0872665;

    double v_trans_max = 0.010;
    double a_trans_max = 0.020;
    double v_rot_max    = 0.0872665;
    double a_rot_max    = 0.174533;

    double t_min = 0.5;
    double max_duration_stretch_factor = 5.0;

    double dq_waypoint_max = 0.002;
    double qdot_pred_max   = 0.15;
    double qddot_pred_max  = 0.30;
    double joint_limit_margin = 0.0872665;

    double feedback_timeout = 0.100;

    double following_position_gate    = 0.002;
    double following_orientation_gate = 0.0174533;
    double following_error_abort_position    = 0.010;
    double following_error_abort_orientation = 0.0872665;
    double following_error_abort_duration    = 0.100;

    double settle_position_threshold    = 0.0005;
    double settle_orientation_threshold = 0.0087265;
    double settle_joint_speed_threshold = 0.01;
    double settle_hold_duration = 0.5;
    double settle_timeout       = 10.0;

    double publish_rate     = 100.0;
    double diagnostics_rate = 10.0;

    // IK safety gate (guide 5.1-5.3)
    double ds_min = 0.01;
    double sigma_threshold    = 0.01;
    double sigma_min_hard_stop = 0.001;
    double ik_lambda_max = 0.01;
    double joint_limit_weight_gain = 50.0;
    double elbow_weight_gain = 0.2;
    double null_space_gain = 0.01;  // k_c
    // Branch-jump guard: reject if ||delta_q - last_accepted_delta_q||
    // exceeds this multiple of dq_waypoint_max.
    double continuity_jump_multiple = 3.0;
};

// Result of one IkSafetyGate::evaluate() call. Appendix A invariant:
// "Returns accepted scale or explicit rejection; no silent clipping."
struct GateResult {
    bool accepted = false;
    Eigen::VectorXd delta_q;      // 7x1, only meaningful if accepted
    double sigma_min = 0.0;
    double lambda    = 0.0;
    double min_joint_margin = 0.0;
    // Linear estimate of how much to shrink the trajectory scale step to
    // fit dq_waypoint_max; caller decides whether/how to use it. Not
    // authoritative since the DLS solve is not exactly linear in s.
    double scale_hint = 1.0;
    std::string reject_reason;
};

}  // namespace trm

#endif  // ULTRASOUND_TRAJECTORY_TRM_TYPES_H
