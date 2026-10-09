#include "ultrasound_trajectory/ros2_support.hpp"
#include <functional>
#include <rclcpp/create_timer.hpp>
/*
 * ultrasound_traj_cartesian_node.cpp
 *
 * Subscribes to /ee_cartesian_target (lwr_controllers/PoseRPY) for a desired
 * absolute end-effector pose expressed directly in the robot BASE frame:
 *   position.x/y/z            -> target x, y, z       [metres]
 *   orientation.roll/pitch/yaw -> target roll,pitch,yaw [radians]  (ZYX)
 *
 * On receipt, executes one smooth (5th-order polynomial) motion from the
 * current EE pose to the requested target pose, then holds the target pose
 * and waits for the next target.
 *
 * The interpolation is paced by a dedicated fixed-rate rclcpp::TimerBase::SharedPtr (not by
 * the arrival rate of /cartesian_position/pose/end_effector), and the
 * polynomial's time argument is recomputed each tick from actual elapsed
 * rclcpp::Time since the motion started -- so a late/skipped timer tick just
 * catches up to the correct point on the curve instead of desynchronizing
 * the whole motion.
 *
 * Orientation is interpolated with quaternion slerp (shortest-path, no
 * gimbal lock), using the same quintic time-scaling as position, rather
 * than interpolating roll/pitch/yaw independently.
 *
 * Every target is sanity-checked before being accepted: displacements beyond
 * ~max_total_linear_dist / ~max_total_angular_dist are rejected outright
 * (catches gross typo/unit errors). Otherwise, if the displacement exceeds
 * ~max_linear_dist / ~max_angular_dist, it is automatically split into a
 * chain of intermediate waypoints (legs), each within that per-leg limit,
 * executed back-to-back as independent smooth motions (zero velocity at
 * every leg boundary). Each leg's own duration is stretched (never
 * shortened) so that neither the linear nor the angular speed exceeds
 * ~max_linear_vel / ~max_angular_vel. The downstream
 * one_task_inverse_kinematics controller does no Cartesian velocity
 * limiting of its own -- this is the only place that happens.
 *
 * Topic in  : /ee_cartesian_target  (lwr_controllers/PoseRPY)  -- target pose, base frame
 * Topic in  : /cartesian_position/pose/end_effector  (lwr_controllers/PoseRPY)  -- current pose
 * Topic out : /lwr/one_task_inverse_kinematics/command (lwr_controllers/PoseRPY)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
//  kuka-lwr Controllers Msgs
#include <lwr_controllers/msg/arm_state.hpp>
#include <lwr_controllers/msg/pose_rpy.hpp>
//  Header Files
#include "ultrasound_trajectory/smooth_traj.h"
//  Eigen lib
#include <Eigen/Core>
#include <Eigen/Geometry>   // AngleAxisd, Quaterniond, rotation-matrix composition

#include <algorithm>
#include <deque>
#include <math.h>
#include <cmath>

using namespace std;

class UltrasoundTrajNode : public rclcpp::Node {

private: // private handle, for parameters only
    rclcpp::Subscription<lwr_controllers::msg::PoseRPY>::SharedPtr sub_pose;    // current EE pose
    rclcpp::Subscription<lwr_controllers::msg::PoseRPY>::SharedPtr sub_target;  // desired absolute target pose (base frame)

    rclcpp::Publisher<lwr_controllers::msg::PoseRPY>::SharedPtr pub_command; // pose command to controller
    rclcpp::TimerBase::SharedPtr control_timer_; // fixed-rate interpolation/publish loop

    lwr_controllers::msg::PoseRPY command_msg;

    // ── current EE state ────────────────────────────────────────────────────
    double ee_x, ee_y, ee_z;
    double ee_roll, ee_pitch, ee_yaw;

    // ── execution state ──────────────────────────────────────────────────────
    bool pose_initialized = false;  // true after first pose callback
    bool target_pending   = false;  // a new target pose has been received
    bool executing        = false;  // currently interpolating a motion

    // One absolute target pose forming a single leg of a (possibly multi-leg)
    // chained motion.
    struct Waypoint {
        double x, y, z;
        Eigen::Quaterniond q;
    };
    std::deque<Waypoint> waypoint_queue_;  // remaining legs, not yet latched
    int leg_index_ = 1;                    // 1-based index of the in-flight leg
    int leg_count_ = 1;                    // total legs in the current chain
                                            // (1 == no splitting, single motion)

    // start / target position for the current motion
    double xs, ys, zs;
    double xt, yt, zt;

    // start / target orientation for the current motion (used for slerp)
    Eigen::Quaterniond q_start_;
    Eigen::Quaterniond q_target_;

    rclcpp::Time motion_start_time_{0, 0, RCL_ROS_TIME};  // set when a latched target starts executing

    // ── trajectory parameters ────────────────────────────────────────────────
    int    SubSteps = 100;  // interpolation steps per second (= control loop rate Hz)
    double T_nom_{0};          // nominal/minimum motion duration [s] (rosparam ~T_nom)
    double T_exec_{0};         // actual duration used for the motion in progress
                            // (>= T_nom_, stretched to respect velocity limits)
    int total_steps_{0};    // T_exec_ * SubSteps, precomputed when a motion starts

    // ── safety limits (rosparams, see constructor for defaults) ──────────────
    double max_linear_vel_{0};   // [m/s]   used to stretch T_exec_ if needed
    double max_angular_vel_{0};  // [rad/s] used to stretch T_exec_ if needed
    double max_linear_dist_{0};  // [m]     per-leg chunk-size limit -- displacement
                              //         beyond this triggers automatic waypoint
                              //         splitting, not rejection
    double max_angular_dist_{0}; // [rad]   per-leg chunk-size limit -- displacement
                              //         beyond this triggers automatic waypoint
                              //         splitting, not rejection
    double max_total_linear_dist_{0};  // [m]   absolute hard-reject ceiling on the
                                     //       *total* requested displacement
    double max_total_angular_dist_{0}; // [rad] absolute hard-reject ceiling on the
                                     //       *total* requested displacement

public:
    UltrasoundTrajNode() : rclcpp::Node("ultrasound_traj_cartesian_node") {
        T_nom_ = ultrasound_trajectory::startup_parameter(*this, "T_nom", 5.0);
        max_linear_vel_ = ultrasound_trajectory::startup_parameter(*this, "max_linear_vel", 0.05);   // 5 cm/s
        max_angular_vel_ = ultrasound_trajectory::startup_parameter(*this, "max_angular_vel", 0.3);    // ~17 deg/s
        max_linear_dist_ = ultrasound_trajectory::startup_parameter(*this, "max_linear_dist", 0.5);    // 50 cm  (per-leg chunk size)
        max_angular_dist_ = ultrasound_trajectory::startup_parameter(*this, "max_angular_dist", 1.57);   // ~90 deg (per-leg chunk size)
        max_total_linear_dist_ = ultrasound_trajectory::startup_parameter(*this, "max_total_linear_dist", 2.0); // 2 m -- absolute reject ceiling
        max_total_angular_dist_ = ultrasound_trajectory::startup_parameter(*this, "max_total_angular_dist", 3.0); // ~172 deg -- absolute reject ceiling

        sub_pose = create_subscription<lwr_controllers::msg::PoseRPY>("/cartesian_position/pose/end_effector", rclcpp::SensorDataQoS(), std::bind(&UltrasoundTrajNode::callback_pose, this, std::placeholders::_1));
        sub_target = create_subscription<lwr_controllers::msg::PoseRPY>("/ee_cartesian_target", rclcpp::SensorDataQoS(), std::bind(&UltrasoundTrajNode::callback_target, this, std::placeholders::_1));

        pub_command = create_publisher<lwr_controllers::msg::PoseRPY>("/lwr/one_task_inverse_kinematics/command", 10);

        control_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / SubSteps), std::bind(&UltrasoundTrajNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "[ultrasound_traj_cartesian_node] Ready. Waiting for first pose...");
    }

    // ── pose callback: just track current EE state ───────────────────────────
    void callback_pose(const lwr_controllers::msg::PoseRPY::ConstSharedPtr& pose_msg) {
        if (!ultrasound_trajectory::finite_pose(*pose_msg)) {return;}

        ee_x     = pose_msg->position.x;
        ee_y     = pose_msg->position.y;
        ee_z     = pose_msg->position.z;
        ee_roll  = pose_msg->orientation.roll;
        ee_pitch = pose_msg->orientation.pitch;
        ee_yaw   = pose_msg->orientation.yaw;

        // ── initialise command_msg once with the robot's starting pose ────────
        if (!pose_initialized) {
            command_msg.id                = 0;   // 0 = full pose (pos + orient)
            command_msg.position.x        = ee_x;
            command_msg.position.y        = ee_y;
            command_msg.position.z        = ee_z;
            command_msg.orientation.roll  = ee_roll;
            command_msg.orientation.pitch = ee_pitch;
            command_msg.orientation.yaw   = ee_yaw;
            pose_initialized = true;
            RCLCPP_INFO(get_logger(), "[ultrasound_traj_cartesian_node] Pose initialised. "
                     "Waiting for /ee_cartesian_target ...");
        }
    }

    // ZYX Euler extraction -- must match KDL::Rotation::GetRPY()
    static void quatToRPY(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
        Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        yaw   = std::atan2( R(1,0),  R(0,0));
        pitch = std::atan2(-R(2,0),  std::sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
        roll  = std::atan2( R(2,1),  R(2,2));
    }

    static Eigen::Quaterniond rpyToQuat(double roll, double pitch, double yaw) {
        // ZYX convention (KDL GetRPY / Rotation::RPY):  R = Rz * Ry * Rx
        return Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));
    }

    // shortest-path angle between two orientations [0, pi]
    static double angleBetween(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
        double d = std::min(1.0, std::abs(a.normalized().dot(b.normalized())));
        return 2.0 * std::acos(d);
    }

    // ── latch a single leg (start pose -> target waypoint) as the motion to
    //    execute next: sets xs/ys/zs, q_start_, xt/yt/zt, q_target_, stretches
    //    this leg's own T_exec_, marks target_pending, and logs. Shared by the
    //    first leg (from callback_target) and every subsequent leg of an
    //    automatically-chained motion (from controlLoop) ─────────────────────
    void latchLeg(double start_x, double start_y, double start_z,
                  const Eigen::Quaterniond& q_start, const Waypoint& target) {
        xs = start_x; ys = start_y; zs = start_z;
        q_start_ = q_start;
        xt = target.x; yt = target.y; zt = target.z;
        q_target_ = target.q;

        double dx = xt - xs, dy = yt - ys, dz = zt - zs;
        double leg_linear_dist  = std::sqrt(dx*dx + dy*dy + dz*dz);
        double leg_angular_dist = angleBetween(q_start_, q_target_);

        // ── stretch (never shorten) the motion duration so neither the linear
        //    nor the angular speed exceeds the configured maximum ─────────────
        T_exec_ = T_nom_;
        if (max_linear_vel_  > 0.0) T_exec_ = std::max(T_exec_, leg_linear_dist  / max_linear_vel_);
        if (max_angular_vel_ > 0.0) T_exec_ = std::max(T_exec_, leg_angular_dist / max_angular_vel_);

        target_pending = true;

        double rs, ps, ys_;
        quatToRPY(q_start_, rs, ps, ys_);
        double rt, pt, yt_;
        quatToRPY(q_target_, rt, pt, yt_);

        if (leg_count_ > 1) {
            RCLCPP_INFO(get_logger(), "[ultrasound_traj_cartesian_node] Leg %d/%d latched "
                     "(dist=%.4f m, angle=%.4f rad, T_exec=%.2f s):",
                     leg_index_, leg_count_, leg_linear_dist, leg_angular_dist, T_exec_);
        } else {
            RCLCPP_INFO(get_logger(), "[ultrasound_traj_cartesian_node] New target received "
                     "(dist=%.4f m, angle=%.4f rad, T_exec=%.2f s):",
                     leg_linear_dist, leg_angular_dist, T_exec_);
        }
        RCLCPP_INFO(get_logger(), "  start  : x=%.4f  y=%.4f  z=%.4f  roll=%.4f  pitch=%.4f  yaw=%.4f",
                 xs, ys, zs, rs, ps, ys_);
        RCLCPP_INFO(get_logger(), "  target : x=%.4f  y=%.4f  z=%.4f  roll=%.4f  pitch=%.4f  yaw=%.4f",
                 xt, yt, zt, rt, pt, yt_);
    }

    // ── target-pose callback ──────────────────────────────────────────────────
    void callback_target(const lwr_controllers::msg::PoseRPY::ConstSharedPtr& msg) {
        if (!ultrasound_trajectory::finite_pose(*msg)) {return;}
        if (!pose_initialized) {
            RCLCPP_WARN(get_logger(), "[ultrasound_traj_cartesian_node] Target received before pose "
                     "is initialised — ignoring.");
            return;
        }
        if (executing) {
            RCLCPP_WARN(get_logger(), "[ultrasound_traj_cartesian_node] Still executing previous "
                     "motion — ignoring new target.");
            return;
        }

        // ── The target pose is given directly in the robot BASE frame ─────────
        // (position and RPY orientation are absolute, not deltas).

        double xs_msg = ee_x, ys_msg = ee_y, zs_msg = ee_z;
        Eigen::Quaterniond q_start = rpyToQuat(ee_roll, ee_pitch, ee_yaw);

        double xt_msg = msg->position.x, yt_msg = msg->position.y, zt_msg = msg->position.z;
        Eigen::Quaterniond q_target =
            rpyToQuat(msg->orientation.roll, msg->orientation.pitch, msg->orientation.yaw);

        double dx = xt_msg - xs_msg, dy = yt_msg - ys_msg, dz = zt_msg - zs_msg;
        double linear_dist  = std::sqrt(dx*dx + dy*dy + dz*dz);
        double angular_dist = angleBetween(q_start, q_target);

        // ── reject targets whose TOTAL displacement exceeds the absolute
        //    safety ceiling, regardless of whether it could be chained ────────
        if (linear_dist > max_total_linear_dist_ || angular_dist > max_total_angular_dist_) {
            RCLCPP_ERROR(get_logger(), "[ultrasound_traj_cartesian_node] Rejecting target: requested "
                      "displacement %.4f m / %.4f rad exceeds absolute safety ceiling "
                      "(max_total_linear_dist=%.4f m, max_total_angular_dist=%.4f rad). "
                      "Target ignored.",
                      linear_dist, angular_dist, max_total_linear_dist_, max_total_angular_dist_);
            return;
        }

        // ── if the displacement exceeds the per-leg chunk size, split it into
        //    N equal-fraction waypoints instead of rejecting it. Straight-line
        //    lerp and slerp both move at exactly uniform fractional distance,
        //    so N = ceil(max(...)) guarantees every leg is within limit ────────
        int n_lin = (max_linear_dist_  > 0.0) ? static_cast<int>(std::ceil(linear_dist  / max_linear_dist_))  : 1;
        int n_ang = (max_angular_dist_ > 0.0) ? static_cast<int>(std::ceil(angular_dist / max_angular_dist_)) : 1;
        int N = std::max(1, std::max(n_lin, n_ang));

        waypoint_queue_.clear();
        leg_index_ = 1;
        leg_count_ = N;

        Waypoint first_wp;
        if (N == 1) {
            // No splitting pressure -- use the raw requested target verbatim,
            // bit-identical to today's single-motion behaviour.
            first_wp.x = xt_msg; first_wp.y = yt_msg; first_wp.z = zt_msg;
            first_wp.q = q_target;
        } else {
            double f = 1.0 / static_cast<double>(N);
            first_wp.x = xs_msg + f * dx; first_wp.y = ys_msg + f * dy; first_wp.z = zs_msg + f * dz;
            first_wp.q = q_start.slerp(f, q_target);
        }

        for (int i = 2; i <= N; ++i) {
            Waypoint wp;
            if (i == N) {
                // Final leg: use the raw requested target verbatim so the
                // chain's last commanded pose is bit-exact with the request.
                wp.x = xt_msg; wp.y = yt_msg; wp.z = zt_msg;
                wp.q = q_target;
            } else {
                double f = static_cast<double>(i) / static_cast<double>(N);
                wp.x = xs_msg + f * dx; wp.y = ys_msg + f * dy; wp.z = zs_msg + f * dz;
                wp.q = q_start.slerp(f, q_target);
            }
            waypoint_queue_.push_back(wp);
        }

        if (N > 1) {
            RCLCPP_INFO(get_logger(), "[ultrasound_traj_cartesian_node] Requested displacement %.4f m / "
                     "%.4f rad exceeds per-leg limit (max_linear_dist=%.4f m, "
                     "max_angular_dist=%.4f rad); splitting into %d legs.",
                     linear_dist, angular_dist, max_linear_dist_, max_angular_dist_, N);
        }

        latchLeg(xs_msg, ys_msg, zs_msg, q_start, first_wp);
    }

    // ── fixed-rate control loop (runs at SubSteps Hz, independent of any
    //    incoming topic's arrival rate) ───────────────────────────────────────
    void controlLoop() {
        if (!pose_initialized) {
            return;
        }

        // ── start executing when a new target has been latched ────────────────
        if (target_pending) {
            target_pending     = false;
            executing          = true;
            motion_start_time_ = now();
            total_steps_       = static_cast<int>(std::round(T_exec_ * SubSteps));
            if (leg_count_ > 1) {
                RCLCPP_INFO(get_logger(), "[ultrasound_traj_cartesian_node] Executing leg %d/%d (T=%.2f s) ...",
                         leg_index_, leg_count_, T_exec_);
            } else {
                RCLCPP_INFO(get_logger(), "[ultrasound_traj_cartesian_node] Executing motion (T=%.2f s) ...", T_exec_);
            }
        }

        if (executing) {
            double elapsed = (now() - motion_start_time_).seconds();
            // Recomputed from elapsed time every tick, so a late/skipped tick
            // catches up to the right point on the curve instead of drifting.
            int step_i = static_cast<int>(std::round(elapsed * SubSteps));
            if (step_i > total_steps_) {
                step_i = total_steps_;
            }

            command_msg.position.x = SmoothTraj(xs, xt, T_exec_, SubSteps, step_i);
            command_msg.position.y = SmoothTraj(ys, yt, T_exec_, SubSteps, step_i);
            command_msg.position.z = SmoothTraj(zs, zt, T_exec_, SubSteps, step_i);

            // Same quintic time-scaling as position, applied to slerp so
            // orientation follows the shortest path with zero end-point
            // velocity/acceleration and no Euler gimbal-lock singularity.
            double sigma = SmoothTraj(0.0, 1.0, T_exec_, SubSteps, step_i);
            sigma = std::min(1.0, std::max(0.0, sigma));
            Eigen::Quaterniond q_interp = q_start_.slerp(sigma, q_target_);
            quatToRPY(q_interp, command_msg.orientation.roll,
                      command_msg.orientation.pitch, command_msg.orientation.yaw);

            pub_command->publish(command_msg);

            if (elapsed >= T_exec_) {
                if (!waypoint_queue_.empty()) {
                    Waypoint next = waypoint_queue_.front();
                    waypoint_queue_.pop_front();
                    ++leg_index_;
                    // Open-loop continuation: chain from this leg's own just-
                    // commanded target (xt/yt/zt, q_target_), not fresh ee_*
                    // feedback -- matches this node's existing open-loop design.
                    latchLeg(xt, yt, zt, q_target_, next);
                } else {
                    executing = false;
                    if (leg_count_ > 1) {
                        RCLCPP_INFO(get_logger(), "[ultrasound_traj_cartesian_node] Chain completed (leg %d/%d). "
                                 "Holding target pose. Waiting for next /ee_cartesian_target ...",
                                 leg_index_, leg_count_);
                    } else {
                        RCLCPP_INFO(get_logger(), "[ultrasound_traj_cartesian_node] Motion completed. "
                                 "Holding target pose. Waiting for next /ee_cartesian_target ...");
                    }
                }
            }
        } else {
            // Hold current target pose (no motion)
            pub_command->publish(command_msg);
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<UltrasoundTrajNode>());
    } catch (const std::exception & error) {
        RCLCPP_ERROR(rclcpp::get_logger("ultrasound_traj_cartesian_node"), "%s", error.what());
        result = 1;
    }
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return result;
}
