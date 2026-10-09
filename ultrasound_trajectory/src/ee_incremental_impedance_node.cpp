#include "ultrasound_trajectory/ros2_support.hpp"
#include <functional>
#include <rclcpp/create_timer.hpp>
/*
 * ee_incremental_impedance_node.cpp
 *
 * Subscribes to a relative 6-DoF delta expressed in the end-effector's own
 * frame (same convention as ee_incremental_motion.cpp):
 *   linear.x/y/z  -> dx, dy, dz  [metres]
 *   angular.x/y/z -> droll, dpitch, dyaw  [radians]
 *
 * On receipt, executes one smooth (5th-order polynomial) step from the
 * current EE pose to (current + delta), then holds the target pose and
 * waits for the next action. Unlike ee_incremental_motion.cpp, the target
 * is published as an absolute base-frame lwr_controllers/CartesianImpedancePoint
 * (position/orientation/stiffness/damping/force), so it drives the Cartesian
 * impedance controller instead of the legacy one_task_inverse_kinematics
 * position controller.
 *
 * Topic in  : ~action_topic  (default /ee_action_impedance, geometry_msgs/Twist)
 * Topic in  : ~pose_topic    (default /cartesian_position/pose/end_effector, lwr_controllers/PoseRPY)
 * Topic out : ~command_topic (default /lwr/cartesian_impedance_controller/command, lwr_controllers/CartesianImpedancePoint)
 *
 * NOTE: ~action_topic defaults to a topic distinct from /ee_action (used by
 * ee_incremental_motion.cpp) and ~move (used by tool_relative_motion_node.cpp)
 * on purpose -- those nodes drive a different controller, and this node must
 * never be double-triggered by a command meant for them. Do not run this node
 * at the same time as those unless ~action_topic has been remapped so exactly
 * one node is listening to any given action topic.
 *
 * ── z-axis force control (EE frame) ─────────────────────────────────────────
 * x, y and orientation are always position-controlled from ~action_topic as
 * described above. z can additionally be handed over to a closed-loop force
 * controller that continuously trims the commanded position along the EE's
 * *current* local z-axis so the measured contact force converges to and
 * holds a fixed setpoint (~target_force_z), independent of unknown
 * tissue/surface stiffness. This is an admittance-style outer loop: it reads
 * real force feedback (~force_topic) and adjusts x_fri.position accordingly;
 * it does NOT use the f_fri feedforward force channel, to keep the loop
 * single-channel and easy to reason about/tune.
 *
 * SAFETY: force control starts DISABLED and must be explicitly turned on via
 * the ~enable_force_control (std_srvs/SetBool) service. It auto-disables
 * (fault) on stale force feedback, on exceeding ~max_allowed_force_z, or on
 * exceeding ~max_force_travel from where it was enabled. All gains/limits
 * below are placeholders -- verify sign and magnitude on your actual sensor
 * and mechanical setup, at very low ~max_force_step_rate, before relying on
 * this for real tissue contact.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <std_srvs/srv/set_bool.hpp>
//  kuka-lwr Controllers Msgs
#include <lwr_controllers/msg/pose_rpy.hpp>
#include <lwr_controllers/msg/cartesian_impedance_point.hpp>
//  Eigen lib
#include <Eigen/Core>
#include <Eigen/Geometry>   // AngleAxisd, rotation-matrix composition

#include <math.h>
#include <cmath>
#include <string>
#include <algorithm>

using namespace std;

class EEIncrementalImpedanceNode : public rclcpp::Node {

private:
    rclcpp::Subscription<lwr_controllers::msg::PoseRPY>::SharedPtr sub_pose;    // current EE pose
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_action;  // incoming 6-DoF action
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_force;   // measured force feedback (z-axis force control)
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_force;

    rclcpp::Publisher<lwr_controllers::msg::CartesianImpedancePoint>::SharedPtr pub_command; // impedance controller command

    lwr_controllers::msg::CartesianImpedancePoint command_msg;
    string frame_id = "0";       // non-empty => x_fri is an absolute pose in this frame

    // ── current EE state (base frame, from /cartesian_position/pose/end_effector) ──
    double ee_x = 0.0, ee_y = 0.0, ee_z = 0.0;
    double ee_roll = 0.0, ee_pitch = 0.0, ee_yaw = 0.0;

    // ── step execution state ─────────────────────────────────────────────────
    bool pose_initialized = false;  // true after first pose callback
    bool action_pending   = false;  // a new action has been latched
    bool executing        = false;  // currently interpolating a step

    // start / target position for the current step
    double xs = 0.0, ys = 0.0, zs = 0.0;
    double xt = 0.0, yt = 0.0, zt = 0.0;

    // start/target orientation, interpolated via slerp (avoids Euler-angle
    // wraparound and gimbal-lock artefacts that independent roll/pitch/yaw
    // interpolation is prone to)
    Eigen::Quaterniond q_start, q_target;

    // wall-clock time the current step began; timing is driven off elapsed
    // time, not a per-callback counter, so it stays correct regardless of the
    // actual pose-topic publish rate
    rclcpp::Time step_start_time{0, 0, RCL_ROS_TIME};

    // ── trajectory / controller parameters (rosparam-overridable) ─────────────
    double T = 6.0;                 // step duration [s], real elapsed time
    double stiffness_linear  = 1500.0;
    double stiffness_angular = 250.0;
    double damping           = 0.8;

    // ── safety limits (rosparam-overridable) ───────────────────────────────────
    // The incoming action is silently clamped to these magnitudes; direction
    // is preserved. Kept more conservative than ee_incremental_motion.cpp's
    // live limits since this node has not yet been validated on hardware.
    double max_linear_step  = 0.05;  // [m]
    double max_angular_step = 0.2;   // [rad]

    // ── z-axis force control state (rosparam-overridable) ──────────────────────
    bool   force_control_enabled = false; // gated by ~enable_force_control service
    bool   force_fault           = false; // set by an auto-disable safety trip
    bool   force_feedback_received = false;
    double measured_force_z = 0.0;
    rclcpp::Time last_force_msg_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_force_tick_time{0, 0, RCL_ROS_TIME};        // for dt in the pose-callback-driven loop
    double force_integral = 0.0;
    // cumulative position trim along the EE's evolving local z-axis, expressed
    // in the base frame -- persists (frozen) across enable/disable so the
    // physically-achieved contact indentation is never snapped back, only
    // actively adjusted while enabled
    Eigen::Vector3d force_trim_offset = Eigen::Vector3d::Zero();

    // Target/gains -- MUST be verified on the real sensor/mechanism before use.
    double target_force_z      = 0.0;    // [N] fixed aim value, EE-frame z-axis
    double force_sign          = 1.0;    // flip to +-1 to correct loop direction w/o rebuilding
    double force_kp            = 0.0005; // [ (m/s) / N ]
    double force_ki            = 0.0;    // [ (m/s) / (N*s) ]
    double max_force_step_rate = 0.003;  // [m/s] velocity cap on the trim
    double max_force_travel    = 0.02;   // [m] hard cap on cumulative trim norm
    double max_allowed_force_z = 8.0;    // [N] hard safety cutoff on |measured force|
    double force_feedback_timeout = 0.5; // [s] treat feedback as stale beyond this
    double stiffness_z_active  = 300.0;  // k_fri.z while force control is enabled (compliant)

public:
    EEIncrementalImpedanceNode() : rclcpp::Node("ee_incremental_impedance_node") {
        string pose_topic, action_topic, command_topic, force_topic;
        pose_topic = ultrasound_trajectory::startup_parameter(*this, "pose_topic", std::string("/cartesian_position/pose/end_effector"));
        action_topic = ultrasound_trajectory::startup_parameter(*this, "action_topic", std::string("/ee_action_impedance"));
        command_topic = ultrasound_trajectory::startup_parameter(*this, "command_topic", std::string("/lwr/cartesian_impedance_controller/command"));
        force_topic = ultrasound_trajectory::startup_parameter(*this, "force_topic", std::string("/Force_calibrated"));

        T = ultrasound_trajectory::startup_parameter(*this, "step_duration", T);
        stiffness_linear = ultrasound_trajectory::startup_parameter(*this, "stiffness_linear", stiffness_linear);
        stiffness_angular = ultrasound_trajectory::startup_parameter(*this, "stiffness_angular", stiffness_angular);
        damping = ultrasound_trajectory::startup_parameter(*this, "damping", damping);
        max_linear_step = ultrasound_trajectory::startup_parameter(*this, "max_linear_step", max_linear_step);
        max_angular_step = ultrasound_trajectory::startup_parameter(*this, "max_angular_step", max_angular_step);

        target_force_z = ultrasound_trajectory::startup_parameter(*this, "target_force_z", target_force_z);
        force_sign = ultrasound_trajectory::startup_parameter(*this, "force_sign", force_sign);
        force_kp = ultrasound_trajectory::startup_parameter(*this, "force_kp", force_kp);
        force_ki = ultrasound_trajectory::startup_parameter(*this, "force_ki", force_ki);
        max_force_step_rate = ultrasound_trajectory::startup_parameter(*this, "max_force_step_rate", max_force_step_rate);
        max_force_travel = ultrasound_trajectory::startup_parameter(*this, "max_force_travel", max_force_travel);
        max_allowed_force_z = ultrasound_trajectory::startup_parameter(*this, "max_allowed_force_z", max_allowed_force_z);
        force_feedback_timeout = ultrasound_trajectory::startup_parameter(*this, "force_feedback_timeout", force_feedback_timeout);
        stiffness_z_active = ultrasound_trajectory::startup_parameter(*this, "stiffness_z_active", stiffness_z_active);

        if (!std::isfinite(T) || T <= 0.0 || !std::isfinite(force_feedback_timeout) ||
            force_feedback_timeout <= 0.0 || max_linear_step <= 0.0 || max_angular_step <= 0.0) {
            throw std::invalid_argument("Invalid duration or feedback/step limits");
        }
        sub_pose = create_subscription<lwr_controllers::msg::PoseRPY>(pose_topic, rclcpp::SensorDataQoS(), std::bind(&EEIncrementalImpedanceNode::callback_pose, this, std::placeholders::_1));
        sub_action = create_subscription<geometry_msgs::msg::Twist>(action_topic, rclcpp::QoS(1), std::bind(&EEIncrementalImpedanceNode::callback_action, this, std::placeholders::_1));
        sub_force = create_subscription<geometry_msgs::msg::Wrench>(force_topic, rclcpp::SensorDataQoS(), std::bind(&EEIncrementalImpedanceNode::callback_force, this, std::placeholders::_1));

        srv_enable_force = create_service<std_srvs::srv::SetBool>("~/enable_force_control",
            [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                handle_enable_force_control(*request, *response);
            });

        pub_command = create_publisher<lwr_controllers::msg::CartesianImpedancePoint>(command_topic, 10);

        RCLCPP_INFO(get_logger(), "[ee_incremental_impedance_node] Ready. pose_topic=%s action_topic=%s "
                 "command_topic=%s force_topic=%s. Force control DISABLED at startup -- "
                 "call ~enable_force_control to arm it. Waiting for first pose...",
                 pose_topic.c_str(), action_topic.c_str(), command_topic.c_str(), force_topic.c_str());
    }

    // ── force feedback callback ─────────────────────────────────────────────────
    void callback_force(const geometry_msgs::msg::Wrench::ConstSharedPtr& force_msg) {
        // Same convention as linear_traj_node.cpp: force.z read directly, no
        // rotation applied -- CartesianImpedancePoint.msg documents f_fri (and,
        // by the same sensor mounting, this feedback) as expressed in the
        // current tip/EE frame already.
        if (!std::isfinite(force_msg->force.z)) {return;}
        measured_force_z = force_msg->force.z;
        last_force_msg_time = now();
        force_feedback_received = true;
    }

    // ── enable/disable service ──────────────────────────────────────────────────
    bool handle_enable_force_control(std_srvs::srv::SetBool::Request& req,
                                      std_srvs::srv::SetBool::Response& res) {
        if (req.data) {
            if (!pose_initialized) {
                res.success = false;
                res.message = "pose not initialised yet";
                return true;
            }
            if (!force_feedback_received ||
                (now() - last_force_msg_time).seconds() > force_feedback_timeout) {
                res.success = false;
                res.message = "no recent force feedback -- refusing to enable";
                return true;
            }
            force_fault           = false;
            force_integral         = 0.0;
            force_control_enabled = true;
            res.success = true;
            res.message = "force control enabled";
            RCLCPP_WARN(get_logger(), "[ee_incremental_impedance_node] Force control ENABLED. "
                     "target_force_z=%.3f N force_sign=%.0f", target_force_z, force_sign);
        } else {
            force_control_enabled = false;
            force_fault            = false;
            res.success = true;
            res.message = "force control disabled";
            RCLCPP_WARN(get_logger(), "[ee_incremental_impedance_node] Force control DISABLED. "
                     "Trim held at current value (no snap-back).");
        }
        return true;
    }

    // ── action callback ───────────────────────────────────────────────────────
    void callback_action(const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
        if (!ultrasound_trajectory::finite_twist(*msg)) {return;}
        if (!pose_initialized) {
            RCLCPP_WARN(get_logger(), "[ee_incremental_impedance_node] Action received before pose is "
                     "initialised — ignoring.");
            return;
        }
        if (executing) {
            RCLCPP_WARN(get_logger(), "[ee_incremental_impedance_node] Still executing previous step — "
                     "ignoring new action.");
            return;
        }

        // ── All 6-DoF deltas are expressed in the EE's local frame ────────────
        //
        // Linear: rotate the local displacement into world/base coordinates.
        //   p_world_delta = R_current * [dx, dy, dz]ᵀ
        //
        // Angular: post-multiply the orientation (delta applied in EE frame).
        //   R_new = R_current * R_delta
        //
        // ZYX convention (KDL GetRPY / Rotation::RPY):  R = Rz * Ry * Rx

        xs = ee_x;
        ys = ee_y;
        zs = ee_z;

        // Current EE rotation matrix (ZYX: R = Rz * Ry * Rx)
        Eigen::Matrix3d R_cur =
            (Eigen::AngleAxisd(ee_yaw,   Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(ee_pitch, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(ee_roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();

        q_start = Eigen::Quaterniond(R_cur).normalized();

        // ── Safety clamping ───────────────────────────────────────────────────
        // Linear: preserve direction, clamp magnitude to max_linear_step
        Eigen::Vector3d dp_local(msg->linear.x, msg->linear.y, msg->linear.z);
        {
            double lin_norm = dp_local.norm();
            if (lin_norm > max_linear_step) {
                dp_local *= max_linear_step / lin_norm;
                RCLCPP_WARN(get_logger(), "[ee_incremental_impedance_node] Linear step clamped: "
                         "%.4f m -> %.4f m (max_linear_step)", lin_norm, max_linear_step);
            }
        }

        // Angular: preserve direction, clamp magnitude to max_angular_step
        Eigen::Vector3d da(msg->angular.x, msg->angular.y, msg->angular.z);
        {
            double ang_norm = da.norm();
            if (ang_norm > max_angular_step) {
                da *= max_angular_step / ang_norm;
                RCLCPP_WARN(get_logger(), "[ee_incremental_impedance_node] Angular step clamped: "
                         "%.4f rad -> %.4f rad (max_angular_step)", ang_norm, max_angular_step);
            }
        }

        if (force_control_enabled && std::abs(dp_local(2)) > 1e-9) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[ee_incremental_impedance_node] Force control is "
                     "active -- ignoring the z component of this action (dz=%.4f). "
                     "z is governed by the force loop, not by ~action_topic, while enabled.",
                     dp_local(2));
        }
        // z is always still position-controlled while force control is disabled,
        // and always computed here regardless (force_trim_offset is added on top
        // at publish time, not folded into xt/yt/zt) -- see callback_pose().

        // ── Position target: map local displacement to base frame ─────────────
        Eigen::Vector3d dp_world = R_cur * dp_local;
        xt = xs + dp_world(0);
        yt = ys + dp_world(1);
        zt = zs + dp_world(2);

        // ── Orientation target: compose rotations in EE frame ─────────────────
        Eigen::Matrix3d R_delta =
            (Eigen::AngleAxisd(da(2), Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(da(1), Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(da(0), Eigen::Vector3d::UnitX())).toRotationMatrix();

        Eigen::Matrix3d R_new = R_cur * R_delta;

        q_target = Eigen::Quaterniond(R_new).normalized();
        // Take the shorter arc: q and -q represent the same rotation, but
        // slerp between them is not equivalent — pick whichever target sign
        // is nearest q_start so we never spin the "long way around".
        if (q_start.dot(q_target) < 0.0) {
            q_target.coeffs() *= -1.0;
        }

        action_pending = true;

        RCLCPP_INFO(get_logger(), "[ee_incremental_impedance_node] New action received (after clamping): "
                 "delta dx=%.4f dy=%.4f dz=%.4f droll=%.4f dpitch=%.4f dyaw=%.4f -> "
                 "target x=%.4f y=%.4f z=%.4f",
                 dp_local(0), dp_local(1), dp_local(2), da(0), da(1), da(2), xt, yt, zt);
    }

    // ── z-axis force control update ─────────────────────────────────────────────
    // Admittance-style outer loop: trims force_trim_offset (base-frame 3D vector)
    // along the EE's *current* local z-axis so measured_force_z converges to
    // target_force_z. Runs once per pose-callback tick. Auto-disables on any
    // safety trip.
    void update_force_control(const rclcpp::Time& now, const Eigen::Matrix3d& R_now) {
        if (!force_control_enabled) {
            return;
        }

        if (!force_feedback_received ||
            (now - last_force_msg_time).seconds() > force_feedback_timeout) {
            RCLCPP_ERROR(get_logger(), "[ee_incremental_impedance_node] Force feedback stale (>%.2f s) -- "
                     "disabling force control.", force_feedback_timeout);
            force_control_enabled = false;
            force_fault            = true;
            return;
        }

        if (std::abs(measured_force_z) > max_allowed_force_z) {
            RCLCPP_ERROR(get_logger(), "[ee_incremental_impedance_node] |measured F_z|=%.2f N exceeds "
                     "max_allowed_force_z=%.2f N -- disabling force control.",
                     measured_force_z, max_allowed_force_z);
            force_control_enabled = false;
            force_fault            = true;
            return;
        }

        double dt = (last_force_tick_time.nanoseconds() == 0) ? 0.0 : (now - last_force_tick_time).seconds();
        last_force_tick_time = now;
        if (dt <= 0.0 || dt > 0.5) {
            // first tick after enabling, or a clock jump -- skip this cycle
            return;
        }

        double e = (target_force_z - measured_force_z) * force_sign;
        force_integral += e * dt;
        double vel = force_kp * e + force_ki * force_integral;
        vel = std::min(std::max(vel, -max_force_step_rate), max_force_step_rate);
        double dz_local = vel * dt;

        Eigen::Vector3d ez_world = R_now.col(2);  // EE's current local z-axis, in base frame
        Eigen::Vector3d candidate = force_trim_offset + dz_local * ez_world;
        if (candidate.norm() > max_force_travel) {
            candidate = candidate.normalized() * max_force_travel;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[ee_incremental_impedance_node] max_force_travel "
                     "(%.4f m) reached -- holding trim, not tracking further.",
                     max_force_travel);
        }
        force_trim_offset = candidate;
    }

    // ── pose callback ────────────────────────────────────────────────────────
    void callback_pose(const lwr_controllers::msg::PoseRPY::ConstSharedPtr& pose_msg) {
        if (!ultrasound_trajectory::finite_pose(*pose_msg)) {return;}

        ee_x     = pose_msg->position.x;
        ee_y     = pose_msg->position.y;
        ee_z     = pose_msg->position.z;
        ee_roll  = pose_msg->orientation.roll;
        ee_pitch = pose_msg->orientation.pitch;
        ee_yaw   = pose_msg->orientation.yaw;

        // ── initialise command_msg once with the robot's starting pose,
        //    stiffness, damping and zero force/torque ──────────────────────────
        if (!pose_initialized) {
            Eigen::Matrix3d R_init =
                (Eigen::AngleAxisd(ee_yaw,   Eigen::Vector3d::UnitZ())
               * Eigen::AngleAxisd(ee_pitch, Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(ee_roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
            Eigen::Quaterniond q_init = Eigen::Quaterniond(R_init).normalized();

            command_msg.header.frame_id = frame_id;
            command_msg.x_fri.position.x = ee_x;
            command_msg.x_fri.position.y = ee_y;
            command_msg.x_fri.position.z = ee_z;
            command_msg.x_fri.orientation.x = q_init.x();
            command_msg.x_fri.orientation.y = q_init.y();
            command_msg.x_fri.orientation.z = q_init.z();
            command_msg.x_fri.orientation.w = q_init.w();

            command_msg.k_fri.x = stiffness_linear;  command_msg.k_fri.y = stiffness_linear;  command_msg.k_fri.z = stiffness_linear;
            command_msg.k_fri.rx = stiffness_angular; command_msg.k_fri.ry = stiffness_angular; command_msg.k_fri.rz = stiffness_angular;

            command_msg.d_fri.x = damping; command_msg.d_fri.y = damping; command_msg.d_fri.z = damping;
            command_msg.d_fri.rx = damping; command_msg.d_fri.ry = damping; command_msg.d_fri.rz = damping;

            command_msg.f_fri.force.x = 0.0;  command_msg.f_fri.force.y = 0.0;  command_msg.f_fri.force.z = 0.0;
            command_msg.f_fri.torque.x = 0.0; command_msg.f_fri.torque.y = 0.0; command_msg.f_fri.torque.z = 0.0;

            // hold at the current pose until the first action arrives
            xs = xt = ee_x; ys = yt = ee_y; zs = zt = ee_z;
            q_start = q_target = q_init;

            pose_initialized = true;
            RCLCPP_INFO(get_logger(), "[ee_incremental_impedance_node] Pose initialised. "
                     "Waiting for action...");
        }

        // ── start executing when a new action has been latched ────────────────
        if (action_pending) {
            action_pending  = false;
            executing       = true;
            step_start_time = now();
            RCLCPP_INFO(get_logger(), "[ee_incremental_impedance_node] Executing step (T=%.2f s) ...", T);
        }

        // ── base x/y/z/orientation target from the position-control step ──────
        double base_x, base_y, base_z;
        Eigen::Quaterniond q_interp;

        if (executing) {
            double elapsed = (now() - step_start_time).seconds();
            double Rho     = elapsed / T;
            bool   finished = Rho >= 1.0;
            Rho = std::min(std::max(Rho, 0.0), 1.0);

            // Same 5th-order polynomial timing law used elsewhere in this
            // package (see smooth_traj.h), parameterised by elapsed-time
            // fraction Rho and shared by both position and orientation slerp.
            double Sigma = 10.0*std::pow(Rho,3) - 15.0*std::pow(Rho,4) + 6.0*std::pow(Rho,5);

            base_x = xs + (xt - xs) * Sigma;
            base_y = ys + (yt - ys) * Sigma;
            base_z = zs + (zt - zs) * Sigma;
            q_interp = q_start.slerp(Sigma, q_target);

            if (finished) {
                executing = false;
                RCLCPP_INFO(get_logger(), "[ee_incremental_impedance_node] Step completed. "
                         "Holding target pose. Waiting for next action...");
            }
        } else {
            // Hold current target pose (no motion)
            base_x = xt; base_y = yt; base_z = zt;
            q_interp = q_target;
        }

        // ── z-axis force control: continuously trims a base-frame offset along
        //    the EE's current local z-axis, added on top of the position target
        //    above. Independent of the executing/holding state -- it runs every
        //    tick so it can hold contact force even while otherwise idle. ──────
        Eigen::Matrix3d R_now =
            (Eigen::AngleAxisd(ee_yaw,   Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(ee_pitch, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(ee_roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
        update_force_control(now(), R_now);

        command_msg.k_fri.z = force_control_enabled ? stiffness_z_active : stiffness_linear;

        command_msg.x_fri.position.x = base_x + force_trim_offset(0);
        command_msg.x_fri.position.y = base_y + force_trim_offset(1);
        command_msg.x_fri.position.z = base_z + force_trim_offset(2);

        command_msg.x_fri.orientation.x = q_interp.x();
        command_msg.x_fri.orientation.y = q_interp.y();
        command_msg.x_fri.orientation.z = q_interp.z();
        command_msg.x_fri.orientation.w = q_interp.w();

        pub_command->publish(command_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<EEIncrementalImpedanceNode>());
    } catch (const std::exception & error) {
        RCLCPP_ERROR(rclcpp::get_logger("ee_incremental_impedance_node"), "%s", error.what());
        result = 1;
    }
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return result;
}
