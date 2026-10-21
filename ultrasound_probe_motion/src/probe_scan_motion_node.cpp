/*
 * probe_scan_motion_node.cpp
 *
 * Combined primary-scan + relative-adjustment motion node for the KUKA LWR
 * ultrasound probe. It is the SOLE publisher on the shared
 * one_task_inverse_kinematics command topic, so it does two jobs in one process
 * with no arbitration / no mux:
 *
 *   1. CRUISE  -- a smooth constant-velocity sweep along the probe frame's +Y
 *      axis (trapezoidal velocity profile). Commanded as a probe-frame velocity
 *      Twist on <cruise_topic>; linear.y is m/s. A value of 0 decelerates to a
 *      stop. <cruise_stopped_topic> (std_msgs/Bool, latched) reports when the
 *      sweep is fully stopped so an orchestrator can hand over safely.
 *
 *   2. STEP   -- a relative 6-DoF probe-frame move (translation m + rotation
 *      rad, ZYX), quintic time law + quaternion SLERP, exactly as
 *      probe_relative_motion_node does. Two channels: <action_topic> (normal,
 *      <step_duration>) and <recovery_action_topic> (slower, <recovery_step_duration>).
 *      Relative moves are accepted only while the node is IDLE (not cruising,
 *      not already stepping) -- the orchestrator stops the cruise first.
 *
 * The probe frame is rigidly attached to the flange (E) frame by the fixed
 * calibration T_E_P = Trans_z(probe_length) * Rot_z(probe_z_rotation_deg) *
 * Trans_y(probe_y_offset), loaded from config/probe_calibration.yaml.
 *
 * Topic in  : <cruise_topic>          geometry_msgs/Twist        -- default /probe_cruise_velocity
 * Topic in  : <action_topic>          geometry_msgs/Twist        -- default /probe_action
 * Topic in  : <recovery_action_topic> geometry_msgs/Twist        -- default /probe_action_recovery
 * Topic in  : <ee_pose_topic>         lwr_controllers/PoseRPY     -- default /cartesian_position/pose/end_effector
 * Topic out : <command_topic>         lwr_controllers/PoseRPY     -- default /lwr/one_task_inverse_kinematics/command
 * Topic out : <cruise_stopped_topic>  std_msgs/Bool (latched)     -- default ~cruise_stopped
 *
 * SAFETY: <command_topic> is a shared legacy interface. Do NOT run this node at
 * the same time as probe_relative_motion_node, ee_incremental_motion,
 * ultrasound_traj_node, tool_relative_motion_node, or a second copy of this
 * node -- only one publisher on that topic may be active.
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "lwr_controllers/msg/pose_rpy.hpp"
#include "ultrasound_probe_motion/action/probe_step.hpp"
#include "ultrasound_probe_motion/quintic_duration.h"
#include "ultrasound_probe_motion/trajectory_progress_guard.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

class ProbeScanMotionNode {

private:
    // ROS 2: one node replaces the public (nh_) and private (pnh_) ROS 1 handles; the private
    // parameters are node parameters
    rclcpp::Node::SharedPtr nh_ = std::make_shared<rclcpp::Node>("probe_scan_motion_node");
    rclcpp::SubscriptionBase::SharedPtr sub_pose_;
    rclcpp::SubscriptionBase::SharedPtr sub_action_;
    rclcpp::SubscriptionBase::SharedPtr sub_recovery_action_;
    rclcpp::SubscriptionBase::SharedPtr sub_cruise_;
    rclcpp::Publisher<lwr_controllers::msg::PoseRPY>::SharedPtr  pub_command_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  pub_cruise_stopped_;
    rclcpp::TimerBase::SharedPtr      control_timer_;

    lwr_controllers::msg::PoseRPY command_msg_;
    Eigen::Isometry3d commanded_ee_pose_ = Eigen::Isometry3d::Identity();

    // ── calibration: flange E -> probe P ────────────────────────────────────
    Eigen::Isometry3d T_E_P_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_P_E_ = Eigen::Isometry3d::Identity();

    // ── current EE state (latched by the pose callback) ─────────────────────
    double ee_x_ = 0, ee_y_ = 0, ee_z_ = 0;
    double ee_roll_ = 0, ee_pitch_ = 0, ee_yaw_ = 0;
    bool pose_initialized_ = false;

    // ── mode ───────────────────────────────────────────────────────────────
    enum class Mode { IDLE, STEPPING, SETTLING, CRUISING };
    Mode mode_ = Mode::IDLE;
    bool cruise_stopped_state_ = true;   // last value published on cruise_stopped

    // ── relative-step state ────────────────────────────────────────────────
    bool action_pending_ = false;
    bool pending_is_recovery_ = false;
    Eigen::Vector3d step_p_start_, step_p_target_;
    Eigen::Quaterniond step_q_start_, step_q_target_;
    rclcpp::Time step_start_time_{0, 0, RCL_ROS_TIME};
    double active_T_ = 1.0;
    double T_ = 1.0;           // ~step_duration
    double recovery_T_ = 1.0;  // ~recovery_step_duration

    typedef ultrasound_probe_motion::action::ProbeStep ProbeStep;
    typedef rclcpp_action::Server<ProbeStep> StepServer;
    typedef rclcpp_action::ServerGoalHandle<ProbeStep> StepGoalHandle;
    StepServer::SharedPtr step_server_;
    std::shared_ptr<StepGoalHandle> step_goal_;
    // ROS 2: a goal can only be reported as canceled after handle_cancel has returned
    std::shared_ptr<StepGoalHandle> cancel_goal_;
    rclcpp::TimerBase::SharedPtr cancel_timer_;
    bool has_step_goal_ = false;
    Eigen::Isometry3d action_probe_target_ = Eigen::Isometry3d::Identity();
    rclcpp::Time action_start_time_{0, 0, RCL_ROS_TIME}, trajectory_complete_time_{0, 0, RCL_ROS_TIME}, in_tolerance_since_{0, 0, RCL_ROS_TIME}, last_feedback_time_{0, 0, RCL_ROS_TIME};
    double action_position_tolerance_m_ = 0.001;
    double action_orientation_tolerance_rad_ = 0.01;
    double action_settle_time_sec_ = 0.25;
    double action_settling_timeout_sec_ = 12.0;
    ultrasound_probe_motion::QuinticDurationLimits adjustment_limits_;
    ultrasound_probe_motion::QuinticDurationLimits local_z_limits_;
    ultrasound_probe_motion::QuinticDurationLimits recovery_limits_;
    bool current_goal_is_recovery_ = false;
    uint8_t action_completion_policy_ = ProbeStep::Goal::POSE_SETTLED;
    double action_max_following_position_error_m_ = 0.005;
    double action_max_following_orientation_error_rad_ = 2.0 * M_PI / 180.0;
    ultrasound_probe_motion::TrajectoryProgressGuard progress_guard_;

    // ── TEMPORARY diagnostic: raw settle-error-vs-time trace ────────────────
    // Set ~settle_debug_csv_path to a writable file path to enable; empty (the
    // default) disables this entirely with negligible overhead. See
    // gpis_probe_adjustment_cruise stall investigation, 2026-09-01.
    std::string settle_debug_csv_path_;
    std::ofstream settle_debug_stream_;

    // ── cruise state ───────────────────────────────────────────────────────
    Eigen::Isometry3d probe_target_ = Eigen::Isometry3d::Identity();  // streamed probe pose
    double cruise_v_ = 0.0;          // current commanded speed [m/s]
    double cruise_v_target_ = 0.0;   // desired speed [m/s]
    double cruise_speed_max_ = 0.02; // ~cruise_speed_max ceiling [m/s]
    double cruise_accel_ = 0.01;     // ~cruise_accel ramp rate [m/s^2]
    double cruise_following_gate_ = 0.005;  // ~cruise_following_gate_m freeze threshold
    double control_rate_ = 100.0;    // ~control_rate_hz
    rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};

    static constexpr double MAX_LINEAR_STEP  = 0.25;   // [m]
    static constexpr double MAX_ANGULAR_STEP = 2.0;    // [rad]
    static constexpr double kCruiseStopEps   = 1e-5;   // [m/s]

public:
    rclcpp::Node::SharedPtr node() { return nh_; }  // ROS 2: node to spin in main()

    ProbeScanMotionNode() {
        double probe_length, probe_y_offset, probe_z_rotation_deg;
        probe_length = nh_->declare_parameter<double>("probe_length", 0.252);
        probe_y_offset = nh_->declare_parameter<double>("probe_y_offset", 0.00775);
        probe_z_rotation_deg = nh_->declare_parameter<double>("probe_z_rotation_deg", 5.0);

        const double theta = probe_z_rotation_deg * M_PI / 180.0;
        const Eigen::Matrix3d R_E_P =
            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Vector3d p_E_P =
            R_E_P * Eigen::Vector3d(0.0, probe_y_offset, 0.0) + Eigen::Vector3d(0.0, 0.0, probe_length);
        T_E_P_.linear() = R_E_P;
        T_E_P_.translation() = p_E_P;
        T_P_E_ = T_E_P_.inverse();

        RCLCPP_INFO(nh_->get_logger(), "[probe_scan_motion_node] Calibration: probe_length=%.5f m, "
                 "probe_y_offset=%.5f m, probe_z_rotation_deg=%.3f deg",
                 probe_length, probe_y_offset, probe_z_rotation_deg);

        std::string action_topic, ee_pose_topic, command_topic, recovery_action_topic;
        std::string cruise_topic, cruise_stopped_topic, step_action_name;
        action_topic = nh_->declare_parameter<std::string>("action_topic", "/probe_action");
        recovery_action_topic = nh_->declare_parameter<std::string>("recovery_action_topic", "/probe_action_recovery");
        cruise_topic = nh_->declare_parameter<std::string>("cruise_topic", "/probe_cruise_velocity");
        cruise_stopped_topic = nh_->declare_parameter<std::string>("cruise_stopped_topic", "cruise_stopped");
        ee_pose_topic = nh_->declare_parameter<std::string>("ee_pose_topic", "/cartesian_position/pose/end_effector");
        command_topic = nh_->declare_parameter<std::string>("command_topic", "/lwr/one_task_inverse_kinematics/command");
        step_action_name = nh_->declare_parameter<std::string>("step_action_name", "probe_step");

        load_duration("step_duration", T_);
        load_duration("recovery_step_duration", recovery_T_);
        loadActionLimits();
        active_T_ = T_;

        load_positive("cruise_speed_max", cruise_speed_max_);
        load_positive("cruise_accel", cruise_accel_);
        load_positive("cruise_following_gate_m", cruise_following_gate_);
        load_positive("control_rate_hz", control_rate_);
        settle_debug_csv_path_ = nh_->declare_parameter<std::string>("settle_debug_csv_path", "");
        if (!settle_debug_csv_path_.empty())
            RCLCPP_WARN(nh_->get_logger(), "[probe_scan_motion_node] TEMPORARY settle debug logging enabled -> %s",
                     settle_debug_csv_path_.c_str());

        sub_pose_   = nh_->create_subscription<lwr_controllers::msg::PoseRPY>(ee_pose_topic, 10, std::bind(&ProbeScanMotionNode::poseCb, this, std::placeholders::_1));
        sub_action_ = nh_->create_subscription<geometry_msgs::msg::Twist>(action_topic, 1, std::bind(&ProbeScanMotionNode::actionCb, this, std::placeholders::_1));
        sub_cruise_ = nh_->create_subscription<geometry_msgs::msg::Twist>(cruise_topic, 1, std::bind(&ProbeScanMotionNode::cruiseCb, this, std::placeholders::_1));
        if (!recovery_action_topic.empty())
            sub_recovery_action_ = nh_->create_subscription<geometry_msgs::msg::Twist>(recovery_action_topic, 1,
                                                 std::bind(&ProbeScanMotionNode::recoveryActionCb, this, std::placeholders::_1));

        pub_command_ = nh_->create_publisher<lwr_controllers::msg::PoseRPY>(command_topic, 10);
        // ROS 1: private (~) latched publisher; ROS 2: ~/ prefix for relative names, transient_local
        pub_cruise_stopped_ = nh_->create_publisher<std_msgs::msg::Bool>(
            (cruise_stopped_topic.empty() || cruise_stopped_topic[0] == '/' || cruise_stopped_topic[0] == '~') ?
                cruise_stopped_topic : "~/" + cruise_stopped_topic,
            rclcpp::QoS(1).transient_local());
        // ROS 2: the ROS 1 goal callback is split into handle_goal (accept/reject) and
        // handle_accepted (start the step); the server starts when it is created
        step_server_ = rclcpp_action::create_server<ProbeStep>(
            nh_, step_action_name,
            std::bind(&ProbeScanMotionNode::stepGoalRequestCb, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ProbeScanMotionNode::stepCancelCb, this, std::placeholders::_1),
            std::bind(&ProbeScanMotionNode::stepGoalCb, this, std::placeholders::_1));
        publishCruiseStopped(true);

        // ROS 1 Timer followed ROS time; rclcpp::create_timer on the node clock does the same
        control_timer_ = rclcpp::create_timer(nh_, nh_->get_clock(), rclcpp::Duration::from_seconds(1.0 / control_rate_),
                                         std::bind(&ProbeScanMotionNode::controlCb, this));

        RCLCPP_INFO(nh_->get_logger(), "[probe_scan_motion_node] Ready. step_duration=%.2f s recovery=%.2f s "
                 "cruise_speed_max=%.4f m/s cruise_accel=%.4f m/s^2 rate=%.0f Hz. "
                 "Waiting for first pose on %s ...",
                 T_, recovery_T_, cruise_speed_max_, cruise_accel_, control_rate_, ee_pose_topic.c_str());
    }

    void load_duration(const std::string& name, double& target) {
        double v = target;
        v = nh_->declare_parameter<double>(name, target);
        if (std::isfinite(v) && v > 0.0) target = v;
        else RCLCPP_WARN(nh_->get_logger(), "[probe_scan_motion_node] ~%s = %.4f is not finite/positive; keeping %.4f",
                      name.c_str(), v, target);
    }
    void load_positive(const std::string& name, double& target) { load_duration(name, target); }

    void loadActionLimits() {
        recovery_limits_.max_linear_velocity_mps = 0.12;
        recovery_limits_.max_linear_acceleration_mps2 = 1.125;
        recovery_limits_.max_angular_velocity_radps = 28.125 * M_PI / 180.0;
        recovery_limits_.max_angular_acceleration_radps2 = 259.81 * M_PI / 180.0;
        double min_duration = adjustment_limits_.minimum_duration_sec;
        load_positive("minimum_action_duration_sec", min_duration);
        adjustment_limits_.minimum_duration_sec = recovery_limits_.minimum_duration_sec = min_duration;
        local_z_limits_ = adjustment_limits_;
        local_z_limits_.minimum_duration_sec = min_duration;
        load_positive("adjustment_max_linear_velocity_mps", adjustment_limits_.max_linear_velocity_mps);
        load_positive("adjustment_max_linear_acceleration_mps2", adjustment_limits_.max_linear_acceleration_mps2);
        local_z_limits_.max_linear_velocity_mps = adjustment_limits_.max_linear_velocity_mps;
        local_z_limits_.max_linear_acceleration_mps2 = adjustment_limits_.max_linear_acceleration_mps2;
        load_positive("recovery_max_linear_velocity_mps", recovery_limits_.max_linear_velocity_mps);
        load_positive("recovery_max_linear_acceleration_mps2", recovery_limits_.max_linear_acceleration_mps2);
        double av = adjustment_limits_.max_angular_velocity_radps * 180.0 / M_PI;
        double aa = adjustment_limits_.max_angular_acceleration_radps2 * 180.0 / M_PI;
        double rv = recovery_limits_.max_angular_velocity_radps * 180.0 / M_PI;
        double ra = recovery_limits_.max_angular_acceleration_radps2 * 180.0 / M_PI;
        load_positive("adjustment_max_angular_velocity_deg_s", av);
        load_positive("adjustment_max_angular_acceleration_deg_s2", aa);
        load_positive("recovery_max_angular_velocity_deg_s", rv);
        load_positive("recovery_max_angular_acceleration_deg_s2", ra);
        double zv = 28.125;
        double za = 259.81;
        load_positive("local_z_max_angular_velocity_deg_s", zv);
        load_positive("local_z_max_angular_acceleration_deg_s2", za);
        adjustment_limits_.max_angular_velocity_radps = av * M_PI / 180.0;
        adjustment_limits_.max_angular_acceleration_radps2 = aa * M_PI / 180.0;
        recovery_limits_.max_angular_velocity_radps = rv * M_PI / 180.0;
        recovery_limits_.max_angular_acceleration_radps2 = ra * M_PI / 180.0;
        local_z_limits_.max_angular_velocity_radps = zv * M_PI / 180.0;
        local_z_limits_.max_angular_acceleration_radps2 = za * M_PI / 180.0;
    }

    // ── pose feedback: latch only ─────────────────────────────────────────
    void poseCb(const lwr_controllers::msg::PoseRPY::ConstSharedPtr m) {
        ee_x_ = m->position.x; ee_y_ = m->position.y; ee_z_ = m->position.z;
        ee_roll_ = m->orientation.roll; ee_pitch_ = m->orientation.pitch; ee_yaw_ = m->orientation.yaw;
        if (!pose_initialized_) {
            const Eigen::Isometry3d ee = currentEePose();
            fillCommandFromEe(ee);
            pose_initialized_ = true;
            RCLCPP_INFO(nh_->get_logger(), "[probe_scan_motion_node] Pose initialised; holding.");
        }
    }

    Eigen::Isometry3d currentEePose() const {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = (Eigen::AngleAxisd(ee_yaw_,   Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(ee_pitch_, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(ee_roll_,  Eigen::Vector3d::UnitX())).toRotationMatrix();
        T.translation() = Eigen::Vector3d(ee_x_, ee_y_, ee_z_);
        return T;
    }
    Eigen::Isometry3d currentProbePose() const { return currentEePose() * T_E_P_; }

    static bool finitePose(const geometry_msgs::msg::Pose& p) {
        return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
               std::isfinite(p.position.z) && std::isfinite(p.orientation.x) &&
               std::isfinite(p.orientation.y) && std::isfinite(p.orientation.z) &&
               std::isfinite(p.orientation.w);
    }

    static Eigen::Isometry3d poseFromMsg(const geometry_msgs::msg::Pose& p) {
        Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
        out.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
        out.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                         p.orientation.y, p.orientation.z).normalized().toRotationMatrix();
        return out;
    }

    // ROS 2: ROS 1 setRejected(result, reason) sent the reason to the client; an rclcpp_action
    // rejection carries no result, so the reason is logged instead
    rclcpp_action::GoalResponse rejectGoal(const std::string& reason) {
        RCLCPP_WARN(nh_->get_logger(), "[probe_scan_motion_node] Step goal rejected: %s", reason.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::GoalResponse stepGoalRequestCb(const rclcpp_action::GoalUUID&,
                                                  std::shared_ptr<const ProbeStep::Goal> g) {
        ProbeStep::Result result;
        const double qnorm = std::sqrt(g->target_probe_pose.orientation.x * g->target_probe_pose.orientation.x +
                                      g->target_probe_pose.orientation.y * g->target_probe_pose.orientation.y +
                                      g->target_probe_pose.orientation.z * g->target_probe_pose.orientation.z +
                                      g->target_probe_pose.orientation.w * g->target_probe_pose.orientation.w);
        if (!pose_initialized_ || mode_ != Mode::IDLE || action_pending_ || has_step_goal_) {
            result.reason = pose_initialized_ ? "busy" : "pose_not_ready";
            return rejectGoal(result.reason);
        }
        const bool guarded = g->completion_policy ==
            ProbeStep::Goal::TRAJECTORY_COMPLETE_GUARDED;
        const bool trajectory_complete = g->completion_policy ==
            ProbeStep::Goal::TRAJECTORY_COMPLETE;
        const bool local_z_profile = g->motion_profile ==
            ProbeStep::Goal::LOCAL_Z_PROFILE;
        const bool fixed_duration_profile = g->motion_profile ==
            ProbeStep::Goal::FIXED_DURATION_PROFILE;
        if (!finitePose(g->target_probe_pose) || qnorm < 1e-9 ||
            (g->completion_policy != ProbeStep::Goal::POSE_SETTLED &&
             !guarded && !trajectory_complete) ||
            (g->motion_profile != ProbeStep::Goal::DEFAULT_PROFILE &&
             !local_z_profile && !fixed_duration_profile) ||
            !(std::isfinite(g->position_tolerance_m) && g->position_tolerance_m > 0.0 &&
              std::isfinite(g->orientation_tolerance_rad) && g->orientation_tolerance_rad > 0.0 &&
              std::isfinite(g->settle_time_sec) && g->settle_time_sec >= 0.0 &&
              std::isfinite(g->settling_timeout_sec) && g->settling_timeout_sec > 0.0) ||
            (guarded && !(std::isfinite(g->max_following_position_error_m) &&
                          g->max_following_position_error_m > 0.0 &&
                          std::isfinite(g->max_following_orientation_error_rad) &&
                          g->max_following_orientation_error_rad > 0.0))) {
            result.reason = "invalid_goal"; return rejectGoal(result.reason);
        }

        action_probe_target_ = poseFromMsg(g->target_probe_pose);
        // Trajectory-complete goals can be chained before measured feedback has
        // caught up. Start them at the held command to avoid stepping the
        // controller target backwards between acknowledged goals.
        const Eigen::Isometry3d start_ee = (guarded || trajectory_complete) ?
            commanded_ee_pose_ : currentEePose();
        const Eigen::Isometry3d start_probe = start_ee * T_E_P_;
        const Eigen::Isometry3d requested_delta = start_probe.inverse() * action_probe_target_;
        if (requested_delta.translation().norm() > MAX_LINEAR_STEP ||
            Eigen::AngleAxisd(requested_delta.rotation()).angle() > MAX_ANGULAR_STEP) {
            result.reason = "target_exceeds_step_limit"; return rejectGoal(result.reason);
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void stepGoalCb(const std::shared_ptr<StepGoalHandle> goal) {
        const std::shared_ptr<const ProbeStep::Goal> g = goal->get_goal();
        // ROS 2: same values as computed in stepGoalRequestCb (called in the same executor step)
        const bool guarded = g->completion_policy ==
            ProbeStep::Goal::TRAJECTORY_COMPLETE_GUARDED;
        const bool trajectory_complete = g->completion_policy ==
            ProbeStep::Goal::TRAJECTORY_COMPLETE;
        const bool local_z_profile = g->motion_profile ==
            ProbeStep::Goal::LOCAL_Z_PROFILE;
        const bool fixed_duration_profile = g->motion_profile ==
            ProbeStep::Goal::FIXED_DURATION_PROFILE;
        const Eigen::Isometry3d start_ee = (guarded || trajectory_complete) ?
            commanded_ee_pose_ : currentEePose();
        step_goal_ = goal; has_step_goal_ = true;
        current_goal_is_recovery_ = g->recovery;
        action_position_tolerance_m_ = g->position_tolerance_m;
        action_orientation_tolerance_rad_ = g->orientation_tolerance_rad;
        action_settle_time_sec_ = g->settle_time_sec;
        action_settling_timeout_sec_ = g->settling_timeout_sec;
        action_completion_policy_ = g->completion_policy;
        if (guarded) {
            action_max_following_position_error_m_ = g->max_following_position_error_m;
            action_max_following_orientation_error_rad_ = g->max_following_orientation_error_rad;
        }
        action_start_time_ = nh_->now(); in_tolerance_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        progress_guard_.reset();

        const Eigen::Isometry3d ee_cur = start_ee;
        const Eigen::Isometry3d ee_tgt = action_probe_target_ * T_P_E_;
        step_p_start_ = ee_cur.translation(); step_p_target_ = ee_tgt.translation();
        step_q_start_ = Eigen::Quaterniond(ee_cur.linear()).normalized();
        step_q_target_ = Eigen::Quaterniond(ee_tgt.linear()).normalized();
        if (step_q_start_.dot(step_q_target_) < 0.0) step_q_target_.coeffs() *= -1.0;
        const double linear_distance = (step_p_target_ - step_p_start_).norm();
        const double angular_distance = Eigen::AngleAxisd(ee_cur.rotation().transpose() * ee_tgt.rotation()).angle();
        if (fixed_duration_profile) active_T_ = T_;
        else {
            const ultrasound_probe_motion::QuinticDurationLimits& limits = g->recovery ?
                recovery_limits_ : (local_z_profile ? local_z_limits_ : adjustment_limits_);
            active_T_ = ultrasound_probe_motion::quinticDurationSec(
                linear_distance, angular_distance, limits);
        }
        trajectory_complete_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        step_start_time_ = action_start_time_; mode_ = Mode::STEPPING;
        RCLCPP_INFO(nh_->get_logger(), "[probe_scan_motion_node] Accepted acknowledged %sstep profile=%s policy=%s (T=%.2f s)",
                 g->recovery ? "recovery " : "",
                 fixed_duration_profile ? "fixed_duration" :
                   (local_z_profile && !g->recovery ? "local_z" : "default"),
                 guarded ? "guarded" : (trajectory_complete ? "trajectory_complete" : "pose_settled"),
                 active_T_);
    }

    rclcpp_action::CancelResponse stepCancelCb(const std::shared_ptr<StepGoalHandle> goal) {
        if (!has_step_goal_ || goal != step_goal_) return rclcpp_action::CancelResponse::REJECT;
        closeSettleDebugStream(false, "cancelled");
        fillCommandFromEe(currentEePose()); mode_ = Mode::IDLE; has_step_goal_ = false;
        // ROS 2: report the goal as canceled once it is in the CANCELING state (next executor cycle)
        cancel_goal_ = goal;
        cancel_timer_ = nh_->create_wall_timer(std::chrono::nanoseconds(0), [this]() {
            cancel_timer_->cancel();
            auto result = std::make_shared<ProbeStep::Result>(); result->reason = "cancelled";
            if (cancel_goal_ && cancel_goal_->is_canceling()) cancel_goal_->canceled(result);
            cancel_goal_.reset();
        });
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void finishStepGoal(bool success, const std::string& reason,
                        double position_error, double orientation_error) {
        auto result = std::make_shared<ProbeStep::Result>();
        result->success = success; result->reason = reason;
        result->final_position_error_m = position_error;
        result->final_orientation_error_rad = orientation_error;
        result->trajectory_duration_sec = active_T_;
        result->settling_duration_sec = (trajectory_complete_time_.nanoseconds() == 0) ? 0.0 :
            std::max(0.0, (nh_->now() - trajectory_complete_time_).seconds());
        result->following_guard_pause_duration_sec = progress_guard_.pausedSec();
        closeSettleDebugStream(success, reason);
        if (success) step_goal_->succeed(result);
        else step_goal_->abort(result);
        has_step_goal_ = false; mode_ = Mode::IDLE;
    }

    // TEMPORARY diagnostic helpers -- see settle_debug_csv_path_ above.
    void openSettleDebugStream() {
        if (settle_debug_csv_path_.empty()) return;
        const bool new_file = !std::ifstream(settle_debug_csv_path_).good();
        settle_debug_stream_.open(settle_debug_csv_path_, std::ios::app);
        if (!settle_debug_stream_.is_open()) {
            RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000, "[probe_scan_motion_node] settle_debug_csv_path unwritable: %s",
                               settle_debug_csv_path_.c_str());
            return;
        }
        if (new_file) settle_debug_stream_ << "elapsed_sec,position_error_mm,orientation_error_deg\n";
        settle_debug_stream_ << "# goal recovery=" << (current_goal_is_recovery_ ? "1" : "0")
                             << " tol_mm=" << action_position_tolerance_m_ * 1000.0
                             << " tol_deg=" << action_orientation_tolerance_rad_ * 180.0 / M_PI
                             << " settle_req_sec=" << action_settle_time_sec_ << '\n';
    }
    void closeSettleDebugStream(bool success, const std::string& reason) {
        if (!settle_debug_stream_.is_open()) return;
        settle_debug_stream_ << "# end success=" << (success ? "1" : "0") << " reason=" << reason << "\n\n";
        settle_debug_stream_.close();
    }

    void fillCommandFromEe(const Eigen::Isometry3d& ee) {
        commanded_ee_pose_ = ee;
        command_msg_.id = 0;
        command_msg_.position.x = ee.translation().x();
        command_msg_.position.y = ee.translation().y();
        command_msg_.position.z = ee.translation().z();
        const Eigen::Matrix3d R = ee.linear();
        command_msg_.orientation.yaw   = std::atan2(R(1, 0), R(0, 0));
        command_msg_.orientation.pitch = std::atan2(-R(2, 0), std::hypot(R(2, 1), R(2, 2)));
        command_msg_.orientation.roll  = std::atan2(R(2, 1), R(2, 2));
    }

    // ── relative-step channels ───────────────────────────────────────────
    void actionCb(const geometry_msgs::msg::Twist::ConstSharedPtr m) { latchAction(m, false); }
    void recoveryActionCb(const geometry_msgs::msg::Twist::ConstSharedPtr m) { latchAction(m, true); }

    void latchAction(const geometry_msgs::msg::Twist::ConstSharedPtr msg, bool is_recovery) {
        if (!pose_initialized_) {
            RCLCPP_WARN(nh_->get_logger(), "[probe_scan_motion_node] Action before first pose -- ignoring.");
            return;
        }
        if (mode_ != Mode::IDLE || action_pending_ || has_step_goal_) {
            RCLCPP_WARN(nh_->get_logger(), "[probe_scan_motion_node] Busy (%s%s) -- ignoring new %saction.",
                     modeName(), action_pending_ ? ", action pending" : "",
                     is_recovery ? "recovery " : "");
            return;
        }

        Eigen::Vector3d dp(msg->linear.x, msg->linear.y, msg->linear.z);
        const double dp_norm = dp.norm();
        if (dp_norm > MAX_LINEAR_STEP) {
            dp *= MAX_LINEAR_STEP / dp_norm;
            RCLCPP_WARN(nh_->get_logger(), "[probe_scan_motion_node] Linear step clamped %.4f -> %.4f m", dp_norm, MAX_LINEAR_STEP);
        }
        Eigen::Vector3d da(msg->angular.x, msg->angular.y, msg->angular.z);
        const double da_norm = da.norm();
        if (da_norm > MAX_ANGULAR_STEP) {
            da *= MAX_ANGULAR_STEP / da_norm;
            RCLCPP_WARN(nh_->get_logger(), "[probe_scan_motion_node] Angular step clamped %.4f -> %.4f rad", da_norm, MAX_ANGULAR_STEP);
        }

        const Eigen::Isometry3d ee_cur = currentEePose();
        const Eigen::Isometry3d probe_cur = ee_cur * T_E_P_;

        Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
        delta.linear() = (Eigen::AngleAxisd(da(2), Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(da(1), Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(da(0), Eigen::Vector3d::UnitX())).toRotationMatrix();
        delta.translation() = dp;

        const Eigen::Isometry3d probe_tgt = probe_cur * delta;   // delta in probe's own frame
        const Eigen::Isometry3d ee_tgt = probe_tgt * T_P_E_;

        step_p_start_ = ee_cur.translation();
        step_p_target_ = ee_tgt.translation();
        step_q_start_ = Eigen::Quaterniond(ee_cur.linear()).normalized();
        step_q_target_ = Eigen::Quaterniond(ee_tgt.linear()).normalized();
        if (step_q_start_.dot(step_q_target_) < 0.0) step_q_target_.coeffs() *= -1.0;

        action_pending_ = true;
        pending_is_recovery_ = is_recovery;
        RCLCPP_INFO(nh_->get_logger(), "[probe_scan_motion_node] Latched %sstep: dp=[%.4f %.4f %.4f] da=[%.4f %.4f %.4f]",
                 is_recovery ? "recovery " : "", dp.x(), dp.y(), dp.z(), da(0), da(1), da(2));
    }

    // ── cruise channel ───────────────────────────────────────────────────
    void cruiseCb(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!pose_initialized_) {
            RCLCPP_WARN(nh_->get_logger(), "[probe_scan_motion_node] Cruise command before first pose -- ignoring.");
            return;
        }
        if (std::abs(msg->linear.x) > 1e-9 || std::abs(msg->linear.z) > 1e-9)
            RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000, "[probe_scan_motion_node] cruise: only linear.y (+Y) is used.");

        double v = msg->linear.y;
        if (mode_ == Mode::STEPPING || mode_ == Mode::SETTLING || action_pending_) {
            if (std::abs(v) > 1e-9)
                RCLCPP_WARN(nh_->get_logger(), "[probe_scan_motion_node] cruise command ignored -- a relative step is %s.",
                         (mode_ == Mode::STEPPING || mode_ == Mode::SETTLING) ? "executing" : "pending");
            return;
        }
        v = std::max(-cruise_speed_max_, std::min(v, cruise_speed_max_));
        cruise_v_target_ = v;

        if (std::abs(v) > 1e-9 && mode_ == Mode::IDLE) {
            probe_target_ = currentProbePose();
            cruise_v_ = 0.0;
            mode_ = Mode::CRUISING;
            publishCruiseStopped(false);
            RCLCPP_INFO(nh_->get_logger(), "[probe_scan_motion_node] Cruise start: v_target=%.4f m/s", v);
        } else if (mode_ == Mode::CRUISING) {
            RCLCPP_INFO(nh_->get_logger(), "[probe_scan_motion_node] Cruise v_target -> %.4f m/s", v);
        }
    }

    // ── fixed-rate control loop ──────────────────────────────────────────
    void controlCb() {
        if (!pose_initialized_) return;

        const rclcpp::Time now = nh_->now();
        double dt = (last_tick_.nanoseconds() == 0) ? (1.0 / control_rate_) : (now - last_tick_).seconds();
        last_tick_ = now;
        dt = std::max(0.0, std::min(dt, 5.0 / control_rate_));   // guard against a stall

        if (action_pending_ && mode_ == Mode::IDLE) {
            action_pending_ = false;
            mode_ = Mode::STEPPING;
            active_T_ = pending_is_recovery_ ? recovery_T_ : T_;
            step_start_time_ = now;
            RCLCPP_INFO(nh_->get_logger(), "[probe_scan_motion_node] Executing %sstep (T=%.2f s) ...",
                     pending_is_recovery_ ? "recovery " : "", active_T_);
        }

        switch (mode_) {
        case Mode::STEPPING: {
            const bool guarded = has_step_goal_ && action_completion_policy_ ==
                ProbeStep::Goal::TRAJECTORY_COMPLETE_GUARDED;
            const bool trajectory_complete = has_step_goal_ && action_completion_policy_ ==
                ProbeStep::Goal::TRAJECTORY_COMPLETE;
            if (guarded) {
                const Eigen::Isometry3d following_error =
                    commanded_ee_pose_.inverse() * currentEePose();
                progress_guard_.update(dt, following_error.translation().norm(),
                    Eigen::AngleAxisd(following_error.rotation()).angle(),
                    action_max_following_position_error_m_,
                    action_max_following_orientation_error_rad_);
                if ((now-action_start_time_).seconds() > active_T_ + action_settling_timeout_sec_) {
                    finishStepGoal(false, "following_error_timeout",
                        (action_probe_target_.translation()-currentProbePose().translation()).norm(),
                        Eigen::AngleAxisd(action_probe_target_.rotation().transpose()*
                                          currentProbePose().rotation()).angle());
                    break;
                }
            }
            double rho = (guarded ? progress_guard_.elapsedSec() :
                          (now - step_start_time_).seconds()) / active_T_;
            const bool finished = rho >= 1.0;
            rho = std::max(0.0, std::min(rho, 1.0));
            const double s = 10.0 * std::pow(rho, 3) - 15.0 * std::pow(rho, 4) + 6.0 * std::pow(rho, 5);

            const Eigen::Vector3d p = step_p_start_ + (step_p_target_ - step_p_start_) * s;
            const Eigen::Matrix3d R = step_q_start_.slerp(s, step_q_target_).toRotationMatrix();
            Eigen::Isometry3d ee = Eigen::Isometry3d::Identity();
            ee.linear() = R; ee.translation() = p;
            fillCommandFromEe(ee);
            pub_command_->publish(command_msg_);

            if (finished) {
                if (has_step_goal_ && (trajectory_complete_time_.nanoseconds() == 0)) {
                    trajectory_complete_time_ = now;
                    openSettleDebugStream();
                }
                if (guarded || trajectory_complete) {
                    const Eigen::Isometry3d error = action_probe_target_.inverse() * currentProbePose();
                    finishStepGoal(true, guarded ? "trajectory_complete_guarded" : "trajectory_complete",
                                   error.translation().norm(),
                                   Eigen::AngleAxisd(error.rotation()).angle());
                } else {
                    mode_ = has_step_goal_ ? Mode::SETTLING : Mode::IDLE;
                }
                RCLCPP_INFO(nh_->get_logger(), "[probe_scan_motion_node] Step trajectory complete; %s.",
                         guarded ? "guarded handoff ready" : trajectory_complete ?
                         "acknowledged trajectory handoff ready" :
                         (has_step_goal_ ? "waiting for measured pose" : "holding"));
            }
            break;
        }
        case Mode::SETTLING: {
            pub_command_->publish(command_msg_);
            const Eigen::Isometry3d error = action_probe_target_.inverse() * currentProbePose();
            const double position_error = error.translation().norm();
            const double orientation_error = Eigen::AngleAxisd(error.rotation()).angle();
            if (settle_debug_stream_.is_open())
                settle_debug_stream_ << (now - trajectory_complete_time_).seconds() << ','
                                     << position_error * 1000.0 << ','
                                     << orientation_error * 180.0 / M_PI << '\n';
            auto feedback = std::make_shared<ProbeStep::Feedback>();
            feedback->phase = "settling"; feedback->position_error_m = position_error;
            feedback->orientation_error_rad = orientation_error;
            if ((last_feedback_time_.nanoseconds() == 0) || (now-last_feedback_time_).seconds() >= 0.1) {
                step_goal_->publish_feedback(feedback); last_feedback_time_ = now;
            }
            if (position_error <= action_position_tolerance_m_ &&
                orientation_error <= action_orientation_tolerance_rad_) {
                if ((in_tolerance_since_.nanoseconds() == 0)) in_tolerance_since_ = now;
                if ((now-in_tolerance_since_).seconds() >= action_settle_time_sec_)
                    finishStepGoal(true, "pose_reached", position_error, orientation_error);
            } else in_tolerance_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            if (has_step_goal_ && !(trajectory_complete_time_.nanoseconds() == 0) &&
                (now-trajectory_complete_time_).seconds() > action_settling_timeout_sec_)
                finishStepGoal(false, "pose_timeout", position_error, orientation_error);
            break;
        }
        case Mode::CRUISING: {
            if (cruise_v_ < cruise_v_target_)
                cruise_v_ = std::min(cruise_v_target_, cruise_v_ + cruise_accel_ * dt);
            else if (cruise_v_ > cruise_v_target_)
                cruise_v_ = std::max(cruise_v_target_, cruise_v_ - cruise_accel_ * dt);

            const double follow_err =
                (currentProbePose().translation() - probe_target_.translation()).norm();
            if (follow_err <= cruise_following_gate_)
                probe_target_.translation() +=
                    cruise_v_ * dt * probe_target_.linear().col(1);   // probe-frame +Y

            fillCommandFromEe(probe_target_ * T_P_E_);
            pub_command_->publish(command_msg_);

            if (std::abs(cruise_v_target_) <= kCruiseStopEps && std::abs(cruise_v_) <= kCruiseStopEps) {
                cruise_v_ = 0.0;
                mode_ = Mode::IDLE;
                publishCruiseStopped(true);
                RCLCPP_INFO(nh_->get_logger(), "[probe_scan_motion_node] Cruise stopped; holding.");
            }
            break;
        }
        case Mode::IDLE:
        default:
            pub_command_->publish(command_msg_);   // hold last pose
            break;
        }
    }

    void publishCruiseStopped(bool stopped) {
        cruise_stopped_state_ = stopped;
        std_msgs::msg::Bool b; b.data = stopped;
        pub_cruise_stopped_->publish(b);
    }

    const char* modeName() const {
        switch (mode_) {
            case Mode::IDLE: return "IDLE";
            case Mode::STEPPING: return "STEPPING";
            case Mode::SETTLING: return "SETTLING";
            case Mode::CRUISING: return "CRUISING";
        }
        return "?";
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ProbeScanMotionNode node;
    rclcpp::spin(node.node());
    rclcpp::shutdown();
    return 0;
}
