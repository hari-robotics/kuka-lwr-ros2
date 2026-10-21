/*
 * probe_relative_motion_node.cpp
 *
 * Commands the KUKA LWR ultrasound probe with relative 6-DoF motions
 * expressed in the PROBE's OWN coordinate frame (not the end-effector
 * frame, not the robot base frame).
 *
 * The probe frame is rigidly attached to the end-effector (flange) frame
 * via a fixed calibration transform T_E_P (flange E -> probe P), loaded
 * from YAML:
 *
 *   T_E_P = Trans_z(probe_length) * Rot_z(probe_z_rotation_deg) * Trans_y(probe_y_offset)
 *
 * On receipt of a relative Twist in the probe frame, this node:
 *   1. computes the current probe pose in the base frame from the current
 *      EE pose feedback and T_E_P,
 *   2. applies the requested delta in the probe's own current frame,
 *   3. converts the resulting target probe pose back to an EE target via
 *      T_E_P^-1,
 *   4. drives the EE smoothly (5th-order polynomial timing + quaternion
 *      SLERP) from the current EE pose to that target, exactly as
 *      ultrasound_trajectory/ee_incremental_motion.cpp does for plain
 *      EE-frame deltas.
 *
 * Topic in  : <action_topic>   (geometry_msgs/Twist)  -- default /probe_action
 * Topic in  : <ee_pose_topic>  (lwr_controllers/PoseRPY) -- default /cartesian_position/pose/end_effector
 * Topic out : <command_topic>  (lwr_controllers/PoseRPY) -- default /lwr/one_task_inverse_kinematics/command
 *
 * SAFETY: <command_topic> is a shared legacy interface. Do not run this
 * node at the same time as ultrasound_trajectory's ee_incremental_motion,
 * ultrasound_traj_node, or tool_relative_motion_node -- they all publish
 * to the same controller topic with no arbitration.
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
//  kuka-lwr Controllers Msgs
#include "lwr_controllers/msg/pose_rpy.hpp"
#include "ultrasound_probe_motion/action/probe_step.hpp"
#include "ultrasound_probe_motion/quintic_duration.h"
//  Eigen lib
#include <Eigen/Core>
#include <Eigen/Geometry>   // AngleAxisd, rotation-matrix composition

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

class ProbeRelativeMotionNode {

private:
    // ROS 2: one node replaces the public (nh_) and private (pnh_) ROS 1 handles; the private
    // parameters are node parameters
    rclcpp::Node::SharedPtr nh_ = std::make_shared<rclcpp::Node>("probe_relative_motion_node");
    rclcpp::SubscriptionBase::SharedPtr sub_pose_;            // current EE pose
    rclcpp::SubscriptionBase::SharedPtr sub_action_;          // incoming 6-DoF probe-frame action
    rclcpp::SubscriptionBase::SharedPtr sub_recovery_action_; // optional slower recovery channel

    rclcpp::Publisher<lwr_controllers::msg::PoseRPY>::SharedPtr  pub_command_; // pose command to controller

    lwr_controllers::msg::PoseRPY command_msg_;

    // ── calibration: flange frame E -> probe frame P, fixed for node lifetime ──
    Eigen::Matrix3d R_E_P_;   // Rot_z(theta)
    Eigen::Vector3d p_E_P_;   // R_E_P_ * [0,d,0] + [0,0,L]
    Eigen::Matrix3d R_P_E_;   // R_E_P_.transpose()
    Eigen::Vector3d p_P_E_;   // -R_P_E_ * p_E_P_

    // ── current EE state ────────────────────────────────────────────────────
    double ee_x_, ee_y_, ee_z_;
    double ee_roll_, ee_pitch_, ee_yaw_;

    // ── step execution state ─────────────────────────────────────────────────
    bool pose_initialized_    = false;  // true after first pose callback
    bool action_pending_      = false;  // a new action has been received
    bool executing_           = false;  // currently interpolating a step
    bool pending_is_recovery_ = false;  // latched action came from the recovery topic
    bool settling_            = false;

    // start pose for the current step (EE frame)
    double xs_, ys_, zs_;

    // target pose for the current step (EE frame; rt_/pt_/yt_ kept for logging only)
    double xt_, yt_, zt_, rt_, pt_, yt2_;

    // start/target orientation (EE frame), interpolated via slerp
    Eigen::Quaterniond q_start_, q_target_;

    // wall-clock time the current step began; timing is driven off elapsed
    // time, not a per-callback counter (see ee_incremental_motion.cpp)
    rclcpp::Time step_start_time_{0, 0, RCL_ROS_TIME};

    // ── trajectory parameters ────────────────────────────────────────────────
    // Step durations [seconds] -- real elapsed wall-clock time, not callback
    // counts. Defaults match ultrasound_probe_impedance's probe_incremental_force_node
    // (3.0 s normal and recovery, commissioned for safety); override via
    // ~step_duration and ~recovery_step_duration (read in the constructor). The
    // recovery duration only applies to actions on ~recovery_action_topic.
    double T_          = 3.0;  // normal step duration
    double recovery_T_ = 3.0;  // duration for steps on the recovery channel
    double active_T_   = 3.0;  // duration of the step currently executing

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
    double action_settling_timeout_sec_ = 9.0;
    ultrasound_probe_motion::QuinticDurationLimits adjustment_limits_;
    ultrasound_probe_motion::QuinticDurationLimits recovery_limits_;

    // ── safety limits ────────────────────────────────────────────────────────
    // The incoming probe-frame action is silently clamped to these
    // magnitudes (direction preserved) before any frame conversion, so the
    // limit means the same thing to the operator regardless of the current
    // calibration rotation or robot orientation.
    static constexpr double MAX_LINEAR_STEP  = 0.25;  // [m]   max translation magnitude per step
    static constexpr double MAX_ANGULAR_STEP = 2;     // [rad] max rotation magnitude per step

public:
    rclcpp::Node::SharedPtr node() { return nh_; }  // ROS 2: node to spin in main()

    ProbeRelativeMotionNode() {
        double probe_length, probe_y_offset, probe_z_rotation_deg;
        probe_length = nh_->declare_parameter<double>("probe_length", 0.252);
        probe_y_offset = nh_->declare_parameter<double>("probe_y_offset", 0.00775);
        probe_z_rotation_deg = nh_->declare_parameter<double>("probe_z_rotation_deg", 5.0);

        double theta = probe_z_rotation_deg * M_PI / 180.0;
        R_E_P_ = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        p_E_P_ = R_E_P_ * Eigen::Vector3d(0.0, probe_y_offset, 0.0)
                 + Eigen::Vector3d(0.0, 0.0, probe_length);
        R_P_E_ = R_E_P_.transpose();
        p_P_E_ = -R_P_E_ * p_E_P_;

        RCLCPP_INFO(nh_->get_logger(), "[probe_relative_motion_node] Calibration: probe_length=%.5f m, "
                 "probe_y_offset=%.5f m, probe_z_rotation_deg=%.3f deg",
                 probe_length, probe_y_offset, probe_z_rotation_deg);
        RCLCPP_INFO(nh_->get_logger(), "[probe_relative_motion_node] p_E_P = [%.5f, %.5f, %.5f]",
                 p_E_P_(0), p_E_P_(1), p_E_P_(2));

        std::string action_topic, ee_pose_topic, command_topic, recovery_action_topic, step_action_name;
        action_topic = nh_->declare_parameter<std::string>("action_topic", "/probe_action");
        ee_pose_topic = nh_->declare_parameter<std::string>("ee_pose_topic", "/cartesian_position/pose/end_effector");
        command_topic = nh_->declare_parameter<std::string>("command_topic", "/lwr/one_task_inverse_kinematics/command");
        recovery_action_topic = nh_->declare_parameter<std::string>("recovery_action_topic",
                                "/probe_action_recovery");
        step_action_name = nh_->declare_parameter<std::string>("step_action_name", "probe_step");

        // Step durations. ~step_duration / ~recovery_step_duration override the
        // 3.0 s defaults independently. A non-finite or non-positive value is
        // rejected with a warning and the default is kept.
        load_duration("step_duration", T_);
        load_duration("recovery_step_duration", recovery_T_);
        loadActionLimits();

        sub_pose_   = nh_->create_subscription<lwr_controllers::msg::PoseRPY>(ee_pose_topic, 10,
                                    std::bind(&ProbeRelativeMotionNode::callback_pose, this, std::placeholders::_1));
        sub_action_ = nh_->create_subscription<geometry_msgs::msg::Twist>(action_topic, 1,
                                    std::bind(&ProbeRelativeMotionNode::callback_action, this, std::placeholders::_1));
        if (!recovery_action_topic.empty()) {
            sub_recovery_action_ = nh_->create_subscription<geometry_msgs::msg::Twist>(recovery_action_topic, 1,
                                    std::bind(&ProbeRelativeMotionNode::callback_recovery_action, this, std::placeholders::_1));
            RCLCPP_INFO(nh_->get_logger(), "[probe_relative_motion_node] Recovery channel enabled on %s (T=%.2f s)",
                     recovery_action_topic.c_str(), recovery_T_);
        }

        pub_command_ = nh_->create_publisher<lwr_controllers::msg::PoseRPY>(command_topic, 10);
        // ROS 2: the ROS 1 goal callback is split into handle_goal (accept/reject) and
        // handle_accepted (start the step); the server starts when it is created
        step_server_ = rclcpp_action::create_server<ProbeStep>(
            nh_, step_action_name,
            std::bind(&ProbeRelativeMotionNode::stepGoalRequestCb, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ProbeRelativeMotionNode::stepCancelCb, this, std::placeholders::_1),
            std::bind(&ProbeRelativeMotionNode::stepGoalCb, this, std::placeholders::_1));

        RCLCPP_INFO(nh_->get_logger(), "[probe_relative_motion_node] Ready (T=%.2f s). Waiting for first pose on %s ...",
                 T_, ee_pose_topic.c_str());
    }

    // Reads a positive, finite step duration from the private parameter `name`
    // into `target`; leaves `target` unchanged (and warns) on bad input.
    void load_duration(const std::string& name, double& target) {
        double v = target;
        v = nh_->declare_parameter<double>(name, target);
        if (std::isfinite(v) && v > 0.0) {
            target = v;
        } else {
            RCLCPP_WARN(nh_->get_logger(), "[probe_relative_motion_node] ~%s = %.4f is not finite/positive; "
                     "keeping %.2f s", name.c_str(), v, target);
        }
    }

    void loadActionLimits() {
        recovery_limits_.max_linear_velocity_mps = 0.12;
        recovery_limits_.max_linear_acceleration_mps2 = 1.125;
        recovery_limits_.max_angular_velocity_radps = 28.125 * M_PI / 180.0;
        recovery_limits_.max_angular_acceleration_radps2 = 259.81 * M_PI / 180.0;
        double minimum = adjustment_limits_.minimum_duration_sec;
        load_duration("minimum_action_duration_sec", minimum);
        adjustment_limits_.minimum_duration_sec = recovery_limits_.minimum_duration_sec = minimum;
        load_duration("adjustment_max_linear_velocity_mps", adjustment_limits_.max_linear_velocity_mps);
        load_duration("adjustment_max_linear_acceleration_mps2", adjustment_limits_.max_linear_acceleration_mps2);
        load_duration("recovery_max_linear_velocity_mps", recovery_limits_.max_linear_velocity_mps);
        load_duration("recovery_max_linear_acceleration_mps2", recovery_limits_.max_linear_acceleration_mps2);
        double av = adjustment_limits_.max_angular_velocity_radps * 180.0 / M_PI;
        double aa = adjustment_limits_.max_angular_acceleration_radps2 * 180.0 / M_PI;
        double rv = recovery_limits_.max_angular_velocity_radps * 180.0 / M_PI;
        double ra = recovery_limits_.max_angular_acceleration_radps2 * 180.0 / M_PI;
        load_duration("adjustment_max_angular_velocity_deg_s", av);
        load_duration("adjustment_max_angular_acceleration_deg_s2", aa);
        load_duration("recovery_max_angular_velocity_deg_s", rv);
        load_duration("recovery_max_angular_acceleration_deg_s2", ra);
        adjustment_limits_.max_angular_velocity_radps = av * M_PI / 180.0;
        adjustment_limits_.max_angular_acceleration_radps2 = aa * M_PI / 180.0;
        recovery_limits_.max_angular_velocity_radps = rv * M_PI / 180.0;
        recovery_limits_.max_angular_acceleration_radps2 = ra * M_PI / 180.0;
    }

    // ── action callbacks ──────────────────────────────────────────────────────
    // Both channels share the same latch logic and the same single-threaded
    // executor + `executing_` guard, so they can never run a step concurrently.
    void callback_action(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        latch_action(msg, /*is_recovery=*/false);
    }

    void callback_recovery_action(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        latch_action(msg, /*is_recovery=*/true);
    }

    Eigen::Isometry3d currentEePose() const {
        Eigen::Isometry3d ee = Eigen::Isometry3d::Identity();
        ee.translation() = Eigen::Vector3d(ee_x_, ee_y_, ee_z_);
        ee.linear() = (Eigen::AngleAxisd(ee_yaw_, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(ee_pitch_, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(ee_roll_, Eigen::Vector3d::UnitX())).toRotationMatrix();
        return ee;
    }

    Eigen::Isometry3d currentProbePose() const {
        Eigen::Isometry3d ep = Eigen::Isometry3d::Identity();
        ep.linear() = R_E_P_; ep.translation() = p_E_P_;
        return currentEePose() * ep;
    }

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
        RCLCPP_WARN(nh_->get_logger(), "[probe_relative_motion_node] Step goal rejected: %s", reason.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::GoalResponse stepGoalRequestCb(const rclcpp_action::GoalUUID&,
                                                  std::shared_ptr<const ProbeStep::Goal> g) {
        ProbeStep::Result result;
        const double qnorm = std::sqrt(g->target_probe_pose.orientation.x * g->target_probe_pose.orientation.x +
                                      g->target_probe_pose.orientation.y * g->target_probe_pose.orientation.y +
                                      g->target_probe_pose.orientation.z * g->target_probe_pose.orientation.z +
                                      g->target_probe_pose.orientation.w * g->target_probe_pose.orientation.w);
        if (!pose_initialized_ || executing_ || settling_ || action_pending_ || has_step_goal_) {
            result.reason = pose_initialized_ ? "busy" : "pose_not_ready";
            return rejectGoal(result.reason);
        }
        if (!finitePose(g->target_probe_pose) || qnorm < 1e-9 ||
            !(std::isfinite(g->position_tolerance_m) && g->position_tolerance_m > 0.0 &&
              std::isfinite(g->orientation_tolerance_rad) && g->orientation_tolerance_rad > 0.0 &&
              std::isfinite(g->settle_time_sec) && g->settle_time_sec >= 0.0 &&
              std::isfinite(g->settling_timeout_sec) && g->settling_timeout_sec > 0.0)) {
            result.reason = "invalid_goal"; return rejectGoal(result.reason);
        }

        action_probe_target_ = poseFromMsg(g->target_probe_pose);
        const Eigen::Isometry3d requested_delta = currentProbePose().inverse() * action_probe_target_;
        if (requested_delta.translation().norm() > MAX_LINEAR_STEP ||
            Eigen::AngleAxisd(requested_delta.rotation()).angle() > MAX_ANGULAR_STEP) {
            result.reason = "target_exceeds_step_limit"; return rejectGoal(result.reason);
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void stepGoalCb(const std::shared_ptr<StepGoalHandle> goal) {
        const std::shared_ptr<const ProbeStep::Goal> g = goal->get_goal();
        step_goal_ = goal; has_step_goal_ = true;
        action_position_tolerance_m_ = g->position_tolerance_m;
        action_orientation_tolerance_rad_ = g->orientation_tolerance_rad;
        action_settle_time_sec_ = g->settle_time_sec;
        action_settling_timeout_sec_ = g->settling_timeout_sec;
        action_start_time_ = nh_->now(); in_tolerance_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

        const Eigen::Isometry3d ee_cur = currentEePose();
        Eigen::Isometry3d pe = Eigen::Isometry3d::Identity();
        pe.linear() = R_P_E_; pe.translation() = p_P_E_;
        const Eigen::Isometry3d ee_tgt = action_probe_target_ * pe;
        xs_ = ee_cur.translation().x(); ys_ = ee_cur.translation().y(); zs_ = ee_cur.translation().z();
        xt_ = ee_tgt.translation().x(); yt_ = ee_tgt.translation().y(); zt_ = ee_tgt.translation().z();
        q_start_ = Eigen::Quaterniond(ee_cur.linear()).normalized();
        q_target_ = Eigen::Quaterniond(ee_tgt.linear()).normalized();
        if (q_start_.dot(q_target_) < 0.0) q_target_.coeffs() *= -1.0;
        const double linear_distance = (ee_tgt.translation() - ee_cur.translation()).norm();
        const double angular_distance = Eigen::AngleAxisd(ee_cur.rotation().transpose() * ee_tgt.rotation()).angle();
        active_T_ = ultrasound_probe_motion::quinticDurationSec(
            linear_distance, angular_distance, g->recovery ? recovery_limits_ : adjustment_limits_);
        trajectory_complete_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        step_start_time_ = action_start_time_; executing_ = true;
        RCLCPP_INFO(nh_->get_logger(), "[probe_relative_motion_node] Accepted acknowledged %sstep (T=%.2f s)",
                 g->recovery ? "recovery " : "", active_T_);
    }

    rclcpp_action::CancelResponse stepCancelCb(const std::shared_ptr<StepGoalHandle> goal) {
        if (!has_step_goal_ || goal != step_goal_) return rclcpp_action::CancelResponse::REJECT;
        executing_ = false; settling_ = false; has_step_goal_ = false;
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
        if (success) step_goal_->succeed(result);
        else step_goal_->abort(result);
        has_step_goal_ = false; settling_ = false;
    }

    void latch_action(const geometry_msgs::msg::Twist::ConstSharedPtr msg, bool is_recovery) {
        if (!pose_initialized_) {
            RCLCPP_WARN(nh_->get_logger(), "[probe_relative_motion_node] Action received before pose is "
                     "initialised -- ignoring.");
            return;
        }
        if (executing_ || settling_ || has_step_goal_) {
            RCLCPP_WARN(nh_->get_logger(), "[probe_relative_motion_node] Still executing previous step -- "
                     "ignoring new action.");
            return;
        }

        // ── All 6-DoF deltas are expressed in the PROBE's own frame ───────────
        xs_ = ee_x_;
        ys_ = ee_y_;
        zs_ = ee_z_;

        // Current EE rotation matrix (ZYX: R = Rz * Ry * Rx, matches KDL RPY)
        Eigen::Matrix3d R_cur =
            (Eigen::AngleAxisd(ee_yaw_,   Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(ee_pitch_, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(ee_roll_,  Eigen::Vector3d::UnitX())).toRotationMatrix();
        Eigen::Vector3d p_cur(ee_x_, ee_y_, ee_z_);

        q_start_ = Eigen::Quaterniond(R_cur).normalized();

        // ── Safety clamping (in the probe frame, before any conversion) ───────
        Eigen::Vector3d dp_local(msg->linear.x, msg->linear.y, msg->linear.z);
        {
            double lin_norm = dp_local.norm();
            if (lin_norm > MAX_LINEAR_STEP) {
                dp_local *= MAX_LINEAR_STEP / lin_norm;
                RCLCPP_WARN(nh_->get_logger(), "[probe_relative_motion_node] Linear step clamped: "
                         "%.4f m -> %.4f m (MAX_LINEAR_STEP)", lin_norm, MAX_LINEAR_STEP);
            }
        }

        Eigen::Vector3d da(msg->angular.x, msg->angular.y, msg->angular.z);
        {
            double ang_norm = da.norm();
            if (ang_norm > MAX_ANGULAR_STEP) {
                da *= MAX_ANGULAR_STEP / ang_norm;
                RCLCPP_WARN(nh_->get_logger(), "[probe_relative_motion_node] Angular step clamped: "
                         "%.4f rad -> %.4f rad (MAX_ANGULAR_STEP)", ang_norm, MAX_ANGULAR_STEP);
            }
        }

        // ── Current probe pose in base frame: T_B_P = T_B_E * T_E_P ───────────
        Eigen::Matrix3d R_B_P_cur = R_cur * R_E_P_;
        Eigen::Vector3d p_B_P_cur = p_cur + R_cur * p_E_P_;

        // ── Delta transform, expressed in the probe's own frame ───────────────
        Eigen::Matrix3d R_delta =
            (Eigen::AngleAxisd(da(2), Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(da(1), Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(da(0), Eigen::Vector3d::UnitX())).toRotationMatrix();

        // ── Target probe pose: delta applied in probe's own current frame ─────
        Eigen::Matrix3d R_B_P_tgt = R_B_P_cur * R_delta;
        Eigen::Vector3d p_B_P_tgt = p_B_P_cur + R_B_P_cur * dp_local;

        // ── Convert target probe pose back to an EE target: T_B_E = T_B_P * T_E_P^-1 ──
        Eigen::Matrix3d R_B_E_tgt = R_B_P_tgt * R_P_E_;
        Eigen::Vector3d p_B_E_tgt = p_B_P_tgt + R_B_P_tgt * p_P_E_;

        xt_ = p_B_E_tgt(0);
        yt_ = p_B_E_tgt(1);
        zt_ = p_B_E_tgt(2);

        // Extract ZYX Euler angles -- must match KDL::Rotation::GetRPY()
        // (kept only for the log line below; interpolation uses q_start_/q_target_)
        yt2_ = std::atan2( R_B_E_tgt(1,0),  R_B_E_tgt(0,0));
        pt_  = std::atan2(-R_B_E_tgt(2,0),  std::sqrt(R_B_E_tgt(2,1)*R_B_E_tgt(2,1)
                                                     + R_B_E_tgt(2,2)*R_B_E_tgt(2,2)));
        rt_  = std::atan2( R_B_E_tgt(2,1),  R_B_E_tgt(2,2));

        q_target_ = Eigen::Quaterniond(R_B_E_tgt).normalized();
        // Take the shorter arc: q and -q represent the same rotation, but
        // slerp between them is not equivalent.
        if (q_start_.dot(q_target_) < 0.0) {
            q_target_.coeffs() *= -1.0;
        }

        action_pending_      = true;
        pending_is_recovery_ = is_recovery;

        RCLCPP_INFO(nh_->get_logger(), "[probe_relative_motion_node] New %sprobe-frame action received (after clamping):",
                 is_recovery ? "recovery " : "");
        RCLCPP_INFO(nh_->get_logger(), "  probe delta : dx=%.4f  dy=%.4f  dz=%.4f  "
                 "droll=%.4f  dpitch=%.4f  dyaw=%.4f",
                 dp_local(0), dp_local(1), dp_local(2),
                 da(0), da(1), da(2));
        RCLCPP_INFO(nh_->get_logger(), "  EE target   : x=%.4f  y=%.4f  z=%.4f  "
                 "roll=%.4f  pitch=%.4f  yaw=%.4f",
                 xt_, yt_, zt_, rt_, pt_, yt2_);
    }

    // ── pose callback (runs at whatever rate ee_pose_topic publishes) ─────────
    void callback_pose(const lwr_controllers::msg::PoseRPY::ConstSharedPtr pose_msg) {

        // Update current EE state
        ee_x_     = pose_msg->position.x;
        ee_y_     = pose_msg->position.y;
        ee_z_     = pose_msg->position.z;
        ee_roll_  = pose_msg->orientation.roll;
        ee_pitch_ = pose_msg->orientation.pitch;
        ee_yaw_   = pose_msg->orientation.yaw;

        // ── initialise command_msg_ once with the robot's starting pose ───────
        if (!pose_initialized_) {
            command_msg_.id                = 0;   // 0 = full pose (pos + orient)
            command_msg_.position.x        = ee_x_;
            command_msg_.position.y        = ee_y_;
            command_msg_.position.z        = ee_z_;
            command_msg_.orientation.roll  = ee_roll_;
            command_msg_.orientation.pitch = ee_pitch_;
            command_msg_.orientation.yaw   = ee_yaw_;
            pose_initialized_ = true;
            RCLCPP_INFO(nh_->get_logger(), "[probe_relative_motion_node] Pose initialised. "
                     "Waiting for probe action ...");
        }

        // ── start executing when a new action has been latched ────────────────
        if (action_pending_) {
            action_pending_  = false;
            executing_       = true;
            active_T_        = pending_is_recovery_ ? recovery_T_ : T_;
            step_start_time_ = nh_->now();
            RCLCPP_INFO(nh_->get_logger(), "[probe_relative_motion_node] Executing %sstep (T=%.2f s) ...",
                     pending_is_recovery_ ? "recovery " : "", active_T_);
        }

        // ── interpolate and publish ────────────────────────────────────────────
        if (executing_) {
            double elapsed  = (nh_->now() - step_start_time_).seconds();
            double rho      = elapsed / active_T_;
            bool   finished = rho >= 1.0;
            rho = std::min(std::max(rho, 0.0), 1.0);

            // Same 5th-order polynomial timing law as ee_incremental_motion.cpp
            double sigma = 10.0*std::pow(rho,3) - 15.0*std::pow(rho,4) + 6.0*std::pow(rho,5);

            command_msg_.position.x = xs_ + (xt_ - xs_) * sigma;
            command_msg_.position.y = ys_ + (yt_ - ys_) * sigma;
            command_msg_.position.z = zs_ + (zt_ - zs_) * sigma;

            // Orientation: slerp, not independent-Euler interpolation.
            Eigen::Quaterniond q_interp = q_start_.slerp(sigma, q_target_);
            Eigen::Matrix3d R_interp = q_interp.toRotationMatrix();

            // Extract ZYX Euler angles -- must match KDL::Rotation::GetRPY()
            command_msg_.orientation.yaw   = std::atan2( R_interp(1,0), R_interp(0,0));
            command_msg_.orientation.pitch = std::atan2(-R_interp(2,0), std::sqrt(R_interp(2,1)*R_interp(2,1)
                                                                                 + R_interp(2,2)*R_interp(2,2)));
            command_msg_.orientation.roll  = std::atan2( R_interp(2,1), R_interp(2,2));

            pub_command_->publish(command_msg_);

            if (finished) {
                executing_ = false;
                settling_ = has_step_goal_;
                if (has_step_goal_ && (trajectory_complete_time_.nanoseconds() == 0))
                    trajectory_complete_time_ = nh_->now();
                RCLCPP_INFO(nh_->get_logger(), "[probe_relative_motion_node] Step trajectory completed; %s.",
                         settling_ ? "waiting for measured pose" : "holding target");
            }
        } else if (settling_) {
            pub_command_->publish(command_msg_);
            const rclcpp::Time now = nh_->now();
            const Eigen::Isometry3d error = action_probe_target_.inverse() * currentProbePose();
            const double position_error = error.translation().norm();
            const double orientation_error = Eigen::AngleAxisd(error.rotation()).angle();
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
        } else {
            // Hold current target pose (no motion)
            pub_command_->publish(command_msg_);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    ProbeRelativeMotionNode node;
    rclcpp::spin(node.node());
    rclcpp::shutdown();
    return 0;
}
