#include "ultrasound_trajectory/ik_safety_gate.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <kdl/jacobian.hpp>

namespace trm {

namespace {
double smoothstep(double v) {
    v = std::min(std::max(v, 0.0), 1.0);
    return 3.0 * v * v - 2.0 * v * v * v;
}

// Floor on lambda so (J*Winv*J^T + lambda^2*I) is always well
// conditioned enough to factor, even far from any singularity.
constexpr double kLambdaFloor = 1.0e-4;
}  // namespace

bool IkSafetyGate::configure(const KDL::Chain& chain,
                              const std::vector<std::string>& joint_names,
                              const std::vector<JointLimit>& joint_limits,
                              const TrmLimits& limits, std::string& error) {
    if (joint_names.size() != chain.getNrOfJoints() ||
        joint_limits.size() != chain.getNrOfJoints()) {
        error = "joint_names/joint_limits size does not match chain.getNrOfJoints()";
        return false;
    }

    chain_ = chain;
    joint_names_ = joint_names;
    joint_limits_ = joint_limits;
    limits_ = limits;

    elbow_index_ = -1;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (joint_names_[i] == "lwr_e1_joint") {
            elbow_index_ = static_cast<int>(i);
            break;
        }
    }

    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));

    have_last_ = false;
    return true;
}

KDL::Frame IkSafetyGate::fk(const KDL::JntArray& q) const {
    KDL::Frame out;
    fk_solver_->JntToCart(q, out);
    return out;
}

Eigen::VectorXd IkSafetyGate::estimateDeltaQ(const KDL::JntArray& q, const KDL::Frame& T_from,
                                              const KDL::Frame& T_to) const {
    const unsigned int n = numJoints();

    KDL::Jacobian J_kdl(n);
    jac_solver_->JntToJac(q, J_kdl);
    const Eigen::MatrixXd& J = J_kdl.data;

    const KDL::Twist twist_diff = KDL::diff(T_from, T_to);
    Eigen::VectorXd delta_x(6);
    delta_x << twist_diff.vel.x(), twist_diff.vel.y(), twist_diff.vel.z(),
        twist_diff.rot.x(), twist_diff.rot.y(), twist_diff.rot.z();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    const Eigen::VectorXd sv = svd.singularValues();
    const double sigma_min = sv(sv.size() - 1);

    const double u = smoothstep(1.0 - sigma_min / limits_.sigma_threshold);
    double lambda = limits_.ik_lambda_max * std::sqrt(u);
    lambda = std::max(lambda, kLambdaFloor);

    Eigen::MatrixXd M = J * J.transpose();
    M.diagonal().array() += lambda * lambda;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
    if (ldlt.info() != Eigen::Success) {
        return Eigen::VectorXd();  // empty: caller must check size() > 0
    }

    const Eigen::VectorXd y = ldlt.solve(delta_x);
    return J.transpose() * y;  // unweighted damped pseudo-inverse estimate
}

void IkSafetyGate::resetContinuity() {
    have_last_ = false;
}

GateResult IkSafetyGate::evaluate(const KDL::JntArray& q_actual, const KDL::Frame& T_actual,
                                   const KDL::Frame& T_candidate, double dt) {
    GateResult result;
    const unsigned int n = numJoints();

    KDL::Jacobian J_kdl(n);
    jac_solver_->JntToJac(q_actual, J_kdl);
    const Eigen::MatrixXd& J = J_kdl.data;  // 6 x n

    const KDL::Twist twist_diff = KDL::diff(T_actual, T_candidate);
    Eigen::VectorXd delta_x(6);
    delta_x << twist_diff.vel.x(), twist_diff.vel.y(), twist_diff.vel.z(),
        twist_diff.rot.x(), twist_diff.rot.y(), twist_diff.rot.z();

    // Smallest singular value of the 6xn Jacobian (guide 5.1's
    // singularity metric). NOTE: J mixes translational [m] and
    // rotational [rad] rows, so sigma_min has mixed units -- a
    // documented simplification (guide does not mandate a
    // characteristic-length correction).
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    const Eigen::VectorXd sv = svd.singularValues();
    const double sigma_min = sv(sv.size() - 1);
    result.sigma_min = sigma_min;

    // Smoothly (C1, no discontinuous threshold per guide 5.1) increasing
    // damping as sigma_min drops below sigma_threshold.
    const double u = smoothstep(1.0 - sigma_min / limits_.sigma_threshold);
    double lambda = limits_.ik_lambda_max * std::sqrt(u);
    lambda = std::max(lambda, kLambdaFloor);
    const double lambda2 = lambda * lambda;
    result.lambda = lambda;

    if (sigma_min < limits_.sigma_min_hard_stop) {
        result.reject_reason = "sigma_min below sigma_min_hard_stop";
        return result;
    }

    // Diagonal weight W: penalize joints inside joint_limit_margin
    // (smoothstep, same shape used for damping above -- "avoid a
    // discontinuous threshold" applied consistently) plus a mild,
    // fixed elbow term.
    Eigen::VectorXd w_inv(n);
    double min_margin = std::numeric_limits<double>::infinity();
    std::vector<double> margin(n);
    for (unsigned int i = 0; i < n; ++i) {
        const double qi = q_actual(i);
        const double d_lower = qi - joint_limits_[i].lower;
        const double d_upper = joint_limits_[i].upper - qi;
        const double d = std::min(d_lower, d_upper);
        margin[i] = d;
        min_margin = std::min(min_margin, d);

        double gamma = 0.0;
        if (d < limits_.joint_limit_margin) {
            const double v = smoothstep(1.0 - d / limits_.joint_limit_margin);
            gamma = limits_.joint_limit_weight_gain * v;
        }
        if (static_cast<int>(i) == elbow_index_) {
            gamma += limits_.elbow_weight_gain;
        }
        w_inv(i) = 1.0 / (1.0 + gamma);
    }
    result.min_joint_margin = min_margin;

    // JW = J * W^-1 (6 x n, W^-1 diagonal so this is a column scale).
    Eigen::MatrixXd JW = J;
    for (unsigned int i = 0; i < n; ++i) {
        JW.col(i) *= w_inv(i);
    }

    Eigen::MatrixXd M = JW * J.transpose();
    M.diagonal().array() += lambda2;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
    if (ldlt.info() != Eigen::Success) {
        result.reject_reason = "damped least-squares solve failed";
        return result;
    }

    const Eigen::VectorXd y = ldlt.solve(delta_x);
    const Eigen::VectorXd delta_q_task = JW.transpose() * y;  // W^-1 J^T y

    // Null-space joint-centering term (guide 5.1: N k_c g(q), "keep
    // k_c small ... must never dominate the Cartesian task").
    const Eigen::MatrixXd Minv = ldlt.solve(Eigen::MatrixXd::Identity(6, 6));
    const Eigen::MatrixXd J_pinv_w = JW.transpose() * Minv;  // n x 6
    const Eigen::MatrixXd N =
        Eigen::MatrixXd::Identity(n, n) - J_pinv_w * J;

    Eigen::VectorXd g(n);
    for (unsigned int i = 0; i < n; ++i) {
        const double lower = joint_limits_[i].lower;
        const double upper = joint_limits_[i].upper;
        const double range = upper - lower;
        const double mid = 0.5 * (upper + lower);
        g(i) = (range > 1e-9) ? -(q_actual(i) - mid) / (range * range) : 0.0;
    }

    const Eigen::VectorXd delta_q = delta_q_task + limits_.null_space_gain * (N * g);
    result.delta_q = delta_q;

    // --- Reject/accept, guide 5.2/5.3/6.1: never silently clip. ---

    // Joint-limit margin: reject only if a joint is already inside the
    // margin AND this step moves it further toward that same limit.
    for (unsigned int i = 0; i < n; ++i) {
        if (margin[i] >= limits_.joint_limit_margin) continue;
        const double d_lower = q_actual(i) - joint_limits_[i].lower;
        const double d_upper = joint_limits_[i].upper - q_actual(i);
        const bool closer_to_lower = d_lower <= d_upper;
        if (closer_to_lower && delta_q(i) < 0.0) {
            result.reject_reason = "joint_limit_margin violated (" + joint_names_[i] + ", lower)";
            return result;
        }
        if (!closer_to_lower && delta_q(i) > 0.0) {
            result.reject_reason = "joint_limit_margin violated (" + joint_names_[i] + ", upper)";
            return result;
        }
    }

    const double max_abs_dq = delta_q.cwiseAbs().maxCoeff();
    if (max_abs_dq > limits_.dq_waypoint_max) {
        result.reject_reason = "dq_waypoint_max exceeded";
        result.scale_hint = limits_.dq_waypoint_max / max_abs_dq;
        return result;
    }

    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(n);
    if (dt > 1e-6) {
        qdot = delta_q / dt;
        const double max_abs_qdot = qdot.cwiseAbs().maxCoeff();
        if (max_abs_qdot > limits_.qdot_pred_max) {
            result.reject_reason = "qdot_pred_max exceeded";
            return result;
        }
    }

    if (have_last_ && dt > 1e-6) {
        const Eigen::VectorXd qddot = (qdot - last_accepted_qdot_) / dt;
        const double max_abs_qddot = qddot.cwiseAbs().maxCoeff();
        if (max_abs_qddot > limits_.qddot_pred_max) {
            result.reject_reason = "qddot_pred_max exceeded";
            return result;
        }
    }

    if (have_last_) {
        const double jump = (delta_q - last_accepted_delta_q_).norm();
        if (jump > limits_.continuity_jump_multiple * limits_.dq_waypoint_max) {
            result.reject_reason = "branch jump vs. last accepted step";
            return result;
        }
    }

    result.accepted = true;
    result.scale_hint = 1.0;
    have_last_ = true;
    last_accepted_delta_q_ = delta_q;
    last_accepted_qdot_ = qdot;
    return result;
}

}  // namespace trm
