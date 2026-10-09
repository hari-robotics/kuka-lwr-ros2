/*
 * ik_safety_gate.h
 *
 * Per-waypoint joint-space safety gate: damped weighted least-squares
 * (DLS) prediction of the joint step needed to reach a candidate
 * Cartesian pose, singularity-aware smooth damping, joint-limit-margin
 * weighting, null-space joint centering, and explicit accept/reject
 * with no silent clipping. No ROS/topic dependency -- pure KDL/Eigen.
 *
 * See: KUKA_LWR4_Tool_Relative_Motion_Development_Guide.md Sections
 * 5.1, 5.2, 5.3, 6, 6.1, Appendix A (IkSafetyGate invariant: "Returns
 * accepted scale or explicit rejection; no silent clipping.").
 */

#ifndef ULTRASOUND_TRAJECTORY_IK_SAFETY_GATE_H
#define ULTRASOUND_TRAJECTORY_IK_SAFETY_GATE_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "ultrasound_trajectory/trm_types.h"

namespace trm {

class IkSafetyGate {
public:
    struct JointLimit {
        double lower = 0.0;
        double upper = 0.0;
        double velocity = 0.0;
    };

    // Builds the FK/Jacobian solvers from chain (copied internally) and
    // stores joint_limits, which must be sized/ordered exactly like
    // the chain's active joints (guide 8.1: "Output is exactly KDL
    // chain order"). Fails if joint_limits.size() != chain's joint
    // count, per guide 8.1's "fail startup if ... seven-joint
    // expectation is wrong" (generalized to whatever the chain
    // actually has, so this class is not hardcoded to seven joints).
    bool configure(const KDL::Chain& chain,
                   const std::vector<std::string>& joint_names,
                   const std::vector<JointLimit>& joint_limits,
                   const TrmLimits& limits, std::string& error);

    KDL::Frame fk(const KDL::JntArray& q) const;

    // One-shot, unweighted damped-pseudo-inverse estimate of the joint
    // motion needed to go directly from T_from to T_to (typically the
    // whole planned move, T_start -> T_goal). NOT a safety decision --
    // no accept/reject, no continuity state touched -- purely a sizing
    // input for Se3Trajectory::ensureMinDuration() so guide 4.3's
    // "lengthen T if ... predicted joint velocity, or predicted joint
    // acceleration [would be violated]" is applied once at plan time
    // instead of relying entirely on the reactive per-tick gate to
    // discover (and spend its stretch budget compensating for) a
    // too-short duration.
    Eigen::VectorXd estimateDeltaQ(const KDL::JntArray& q, const KDL::Frame& T_from,
                                    const KDL::Frame& T_to) const;

    // Core gate (guide 5.1-5.3). q_actual/T_actual are the measured
    // state; T_candidate is the next trajectory sample under
    // consideration; dt is the time since the last *accepted*
    // evaluation (used for qdot/qddot prediction).
    GateResult evaluate(const KDL::JntArray& q_actual, const KDL::Frame& T_actual,
                         const KDL::Frame& T_candidate, double dt);

    // Clears the continuity/qdot history. Call when starting a new
    // motion request (guide 5.3's continuity gate must not compare
    // across unrelated requests) or after a FAULT/HOLD reset.
    void resetContinuity();

    const std::vector<std::string>& jointNames() const { return joint_names_; }
    unsigned int numJoints() const { return static_cast<unsigned int>(joint_names_.size()); }

private:
    KDL::Chain chain_;
    std::vector<std::string> joint_names_;
    std::vector<JointLimit> joint_limits_;
    TrmLimits limits_;
    int elbow_index_ = -1;  // index of lwr_e1_joint within joint_names_, or -1

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;

    bool have_last_ = false;
    Eigen::VectorXd last_accepted_delta_q_;
    Eigen::VectorXd last_accepted_qdot_;
};

}  // namespace trm

#endif  // ULTRASOUND_TRAJECTORY_IK_SAFETY_GATE_H
