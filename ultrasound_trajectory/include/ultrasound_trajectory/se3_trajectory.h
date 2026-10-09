/*
 * se3_trajectory.h
 *
 * Body-fixed (tool-frame) SE(3) goal composition and quintic-scaled
 * interpolation. No ROS/topic dependency -- pure KDL/Eigen in and out --
 * so it can be planned and sampled from a unit test without a ROS master.
 *
 * See: KUKA_LWR4_Tool_Relative_Motion_Development_Guide.md Sections 1.1,
 * 1.3, 4.1, 4.2, 4.3, 5.2 (duration stretch), Appendix A.
 */

#ifndef ULTRASOUND_TRAJECTORY_SE3_TRAJECTORY_H
#define ULTRASOUND_TRAJECTORY_SE3_TRAJECTORY_H

#include <kdl/frames.hpp>
#include <string>

#include "ultrasound_trajectory/trm_types.h"

namespace trm {

class Se3Trajectory {
public:
    // Exact peak factors of the quintic profile s(u)=10u^3-15u^4+6u^5 on
    // u in [0,1]: max(s'(u)) = 15/8 at u=0.5, max(|s''(u)|) = 10/sqrt(3)
    // at u = 0.5 -+ 1/(2*sqrt(3)). Shared with MotionStateMachine so a
    // joint-space duration estimate (guide 4.3: "lengthen T if ...
    // predicted joint velocity, or predicted joint acceleration [would
    // be violated]") uses the exact same closed-form relationship as
    // the Cartesian sizing below, instead of a second, possibly
    // inconsistent, approximation.
    static constexpr double kQuinticPeakVelFactor = 1.875;              // 15/8
    static constexpr double kQuinticPeakAccelFactor = 5.773502691896258;  // 10/sqrt(3)

    // Quintic time-scaling law with zero velocity/acceleration at both
    // ends (guide 4.2): s(u) = 10u^3 - 15u^4 + 6u^5, u clamped to [0,1].
    static double quinticScale(double u);

    // Composes T_goal = T_start * E0_T_E1 in the tool frame frozen at
    // the start of the request (guide 4.1), validates the request
    // against limits, and picks a conservative duration (guide 4.3,
    // closed-form: the quintic profile's exact peak velocity/
    // acceleration factors, 15/8 and 10/sqrt(3), used directly instead
    // of guide 4.3's iterative "sample and lengthen" description --
    // equivalent result, no sampling loop needed for this fixed shape).
    //
    // Returns false (with reject_reason filled) on non-finite input or
    // a request magnitude exceeding request_translation_max /
    // request_rotation_max (guide 4.2: "Reject non-finite input and
    // rotations whose magnitude exceeds the configured per-request
    // limit").
    bool plan(const KDL::Frame& T_start, const MoveRequest& req,
              const TrmLimits& limits, std::string& reject_reason);

    // Position/orientation at scalar progress s in [0,1] (guide 4.2):
    //   p(s) = p_start + R_start * (s * delta_p_tool)
    //   R(s) = R_start * Exp(s * phi_tool), phi_tool = Log(R_delta)
    KDL::Frame sampleAtScale(double s) const;

    // Guide 5.2.4: "lengthen the trajectory rather than clipping
    // individual joints independently." Multiplies the current
    // duration by factor (>1). Returns false without effect if the
    // result would exceed max_duration_stretch_factor * the original
    // planned duration.
    //
    // NOTE on why progress is tracked as a time-fraction u by the
    // caller (MotionStateMachine), not as an absolute-elapsed-time
    // lookup here: stretching T only has to slow *future* per-tick
    // increments of u (each tick nominally advances u by dt/duration()).
    // u itself never needs to be recomputed from absolute elapsed time,
    // so a mid-trajectory stretch can never make u -- and therefore
    // quinticScale(u) -- jump backwards. An elapsed-time/T lookup would
    // not have this property (T growing under a fixed elapsed numerator
    // shrinks u), which is why that approach was deliberately not used.
    bool stretchDuration(double factor);

    // Raises the planned duration to at least t_min_required, and also
    // raises the "original" duration baseline that stretchDuration()'s
    // max_duration_stretch_factor cap is measured against. Unlike
    // stretchDuration(), this is meant to be called once at plan time
    // (guide 4.3), before execution starts, to size T against limits
    // stretchDuration() itself has no visibility into (e.g. the
    // joint-space qdot_pred_max/qddot_pred_max, which need a Jacobian
    // estimate that lives in IkSafetyGate, not here) -- so that the
    // *reactive* per-tick stretch budget isn't spent compensating for a
    // duration that was foreseeably too short from the start.
    void ensureMinDuration(double t_min_required);

    const KDL::Frame& start() const { return T_start_; }
    const KDL::Frame& goal() const { return T_goal_; }
    double duration() const { return T_; }

private:
    KDL::Frame T_start_;
    KDL::Frame T_goal_;
    KDL::Vector delta_p_tool_;  // tool-frame translation, frozen at plan()
    KDL::Vector phi_tool_;      // Log(R_delta), frozen at plan()

    double T_ = 0.0;
    double T_original_ = 0.0;
    double max_duration_stretch_factor_ = 1.0;
};

}  // namespace trm

#endif  // ULTRASOUND_TRAJECTORY_SE3_TRAJECTORY_H
