#include "ultrasound_trajectory/se3_trajectory.h"

#include <algorithm>
#include <cmath>

namespace trm {

namespace {
bool allFinite(std::initializer_list<double> vals) {
    for (double v : vals) {
        if (!std::isfinite(v)) return false;
    }
    return true;
}
}  // namespace

double Se3Trajectory::quinticScale(double u) {
    u = std::min(std::max(u, 0.0), 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    return 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
}

bool Se3Trajectory::plan(const KDL::Frame& T_start, const MoveRequest& req,
                          const TrmLimits& limits, std::string& reject_reason) {
    if (!allFinite({req.dx, req.dy, req.dz, req.droll, req.dpitch, req.dyaw,
                    req.requested_duration})) {
        reject_reason = "non-finite request";
        return false;
    }

    const KDL::Vector dp_tool(req.dx, req.dy, req.dz);
    const double trans_mag = dp_tool.Norm();
    if (trans_mag > limits.request_translation_max) {
        reject_reason = "translation magnitude " + std::to_string(trans_mag) +
                         " exceeds request_translation_max";
        return false;
    }

    // Guide 4.1: R_delta = RotX(droll) * RotY(dpitch) * RotZ(dyaw),
    // post-multiplied onto the start orientation (body-fixed).
    const KDL::Rotation R_delta = KDL::Rotation::RotX(req.droll) *
                                   KDL::Rotation::RotY(req.dpitch) *
                                   KDL::Rotation::RotZ(req.dyaw);

    // Log(R_delta): a vector whose direction is the equivalent axis and
    // whose norm is the equivalent angle (guide 4.2's phi_tool).
    const KDL::Vector phi_tool = R_delta.GetRot();
    const double rot_mag = phi_tool.Norm();
    if (rot_mag > limits.request_rotation_max) {
        reject_reason = "rotation magnitude " + std::to_string(rot_mag) +
                         " exceeds request_rotation_max";
        return false;
    }

    T_start_ = T_start;
    delta_p_tool_ = dp_tool;
    phi_tool_ = phi_tool;

    T_goal_.p = T_start_.p + T_start_.M * dp_tool;
    T_goal_.M = T_start_.M * R_delta;

    double T = std::max(req.requested_duration, limits.t_min);
    if (limits.v_trans_max > 0.0) {
        T = std::max(T, kQuinticPeakVelFactor * trans_mag / limits.v_trans_max);
    }
    if (limits.a_trans_max > 0.0 && trans_mag > 0.0) {
        T = std::max(T, std::sqrt(kQuinticPeakAccelFactor * trans_mag / limits.a_trans_max));
    }
    if (limits.v_rot_max > 0.0) {
        T = std::max(T, kQuinticPeakVelFactor * rot_mag / limits.v_rot_max);
    }
    if (limits.a_rot_max > 0.0 && rot_mag > 0.0) {
        T = std::max(T, std::sqrt(kQuinticPeakAccelFactor * rot_mag / limits.a_rot_max));
    }

    T_ = T;
    T_original_ = T;
    max_duration_stretch_factor_ = limits.max_duration_stretch_factor;
    return true;
}

KDL::Frame Se3Trajectory::sampleAtScale(double s) const {
    s = std::min(std::max(s, 0.0), 1.0);

    KDL::Frame T;
    T.p = T_start_.p + T_start_.M * (delta_p_tool_ * s);

    const KDL::Vector v = phi_tool_ * s;
    const double angle = v.Norm();
    const KDL::Rotation R_s =
        (angle > 1e-12) ? KDL::Rotation::Rot(v, angle) : KDL::Rotation::Identity();
    T.M = T_start_.M * R_s;

    return T;
}

void Se3Trajectory::ensureMinDuration(double t_min_required) {
    if (t_min_required > T_) {
        T_ = t_min_required;
        T_original_ = t_min_required;
    }
}

bool Se3Trajectory::stretchDuration(double factor) {
    if (factor <= 1.0) return true;  // no-op, nothing to stretch
    const double T_new = T_ * factor;
    if (T_new > T_original_ * max_duration_stretch_factor_) {
        return false;
    }
    T_ = T_new;
    return true;
}

}  // namespace trm
