/*
 * probe_incremental_impedance_node.cpp
 *
 * Drives the KUKA LWR Cartesian impedance controller from a relative 6-DoF
 * delta expressed in the ULTRASOUND PROBE's own frame (not the end-effector
 * frame, not the robot base frame):
 *   linear.x/y/z  -> dx, dy, dz  [metres]
 *   angular.x/y/z -> droll, dpitch, dyaw  [radians]
 *
 * The probe frame is rigidly attached to the end-effector (flange) frame via
 * a fixed calibration transform T_E_P (flange E -> probe P), loaded from
 * YAML with the same convention as
 * ultrasound_probe_motion/src/probe_relative_motion_node.cpp:
 *
 *   T_E_P = Trans_z(probe_length) * Rot_z(probe_z_rotation_deg) * Trans_y(probe_y_offset)
 *
 * On receipt of an action this node:
 *   1. computes the current probe pose in the base frame from the current
 *      EE pose feedback and T_E_P,
 *   2. applies the requested delta in the probe's own current frame,
 *   3. converts the resulting target probe pose back to an EE target via
 *      T_E_P^-1 (exactly as probe_relative_motion_node.cpp does),
 *   4. executes one smooth (5th-order polynomial + quaternion SLERP) step
 *      from the current EE pose to that EE target, then holds it -- but,
 *      unlike probe_relative_motion_node.cpp, publishes the target as an
 *      absolute base-frame lwr_controllers/CartesianImpedancePoint
 *      (position/orientation/stiffness/damping/force), so it drives the
 *      Cartesian impedance controller instead of the legacy
 *      one_task_inverse_kinematics position controller. This half of the
 *      node (impedance publishing + optional z-axis force loop) mirrors
 *      ultrasound_trajectory/ee_incremental_impedance_node.cpp exactly.
 *
 * Topic in  : ~action_topic  (default /probe_action_impedance, geometry_msgs/Twist)
 * Topic in  : ~pose_topic    (default /cartesian_position/pose/end_effector, lwr_controllers/PoseRPY)
 * Topic out : ~command_topic (default /lwr/cartesian_impedance_controller/command, lwr_controllers/CartesianImpedancePoint)
 *
 * NOTE: ~action_topic defaults to a topic distinct from /ee_action_impedance
 * (used by ee_incremental_impedance_node.cpp) and /probe_action (used by
 * probe_relative_motion_node.cpp) on purpose -- those nodes drive different
 * controllers/frames, and this node must never be double-triggered by a
 * command meant for them.
 *
 * SAFETY: ~command_topic is shared with ee_incremental_impedance_node.cpp.
 * Do not run this node at the same time as that one unless ~command_topic
 * has been remapped so exactly one node is driving the impedance controller.
 *
 * ── z-axis force control (probe/EE frame) ───────────────────────────────────
 * x, y and orientation are always position-controlled from ~action_topic as
 * described above. z can additionally be handed over to a closed-loop force
 * controller that continuously trims the commanded position along the
 * probe's *current* local z-axis so the measured contact force converges to
 * and holds a fixed setpoint (~target_force_z). This is the same
 * admittance-style outer loop as ee_incremental_impedance_node.cpp, reused
 * unchanged: it reads real force feedback (~force_topic) and adjusts
 * x_fri.position accordingly, without touching the f_fri feedforward
 * channel.
 *
 * The trim axis is computed as R_now.col(2) -- the EE's current local
 * z-axis in base coordinates. This is numerically identical to the probe's
 * current local z-axis here: T_E_P's rotation is a pure Rot_z(theta), and a
 * rotation about z leaves the z basis vector unchanged
 * (R_E_P.col(2) == [0,0,1]), so R_B_P.col(2) == R_B_E.col(2) always. No
 * change from ee_incremental_impedance_node.cpp's force loop is needed for
 * it to remain correct in the probe frame.
 *
 * SAFETY: force control starts DISABLED and must be explicitly turned on via
 * the ~enable_force_control (std_srvs/SetBool) service. It auto-disables
 * (fault) on stale force feedback, on exceeding ~max_allowed_force_z, or on
 * exceeding ~max_force_travel from where it was enabled. All gains/limits
 * below are placeholders -- verify sign and magnitude on your actual sensor
 * and mechanical setup, at very low ~max_force_step_rate, before relying on
 * this for real tissue contact.
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_srvs/srv/set_bool.hpp"
//  kuka-lwr Controllers Msgs
#include "lwr_controllers/msg/pose_rpy.hpp"
#include "lwr_controllers/msg/cartesian_impedance_point.hpp"
//  Eigen lib
#include <Eigen/Core>
#include <Eigen/Geometry>   // AngleAxisd, rotation-matrix composition

#include <math.h>
#include <cmath>
#include <string>
#include <algorithm>

using namespace std;

class ProbeIncrementalImpedanceNode {

private:
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("probe_incremental_impedance_node");
    rclcpp::SubscriptionBase::SharedPtr sub_pose;    // current EE pose
    rclcpp::SubscriptionBase::SharedPtr sub_action;  // incoming 6-DoF probe-frame action
    rclcpp::SubscriptionBase::SharedPtr sub_force;   // measured force feedback (z-axis force control)
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_force;

    rclcpp::Publisher<lwr_controllers::msg::CartesianImpedancePoint>::SharedPtr pub_command; // impedance controller command

    lwr_controllers::msg::CartesianImpedancePoint command_msg;
    string frame_id = "0";       // non-empty => x_fri is an absolute pose in this frame

    // ── calibration: flange frame E -> probe frame P, fixed for node lifetime ──
    Eigen::Matrix3d R_E_P;   // Rot_z(theta)
    Eigen::Vector3d p_E_P;   // R_E_P * [0,d,0] + [0,0,L]
    Eigen::Matrix3d R_P_E;   // R_E_P.transpose()
    Eigen::Vector3d p_P_E;   // -R_P_E * p_E_P

    // ── current EE state (base frame, from /cartesian_position/pose/end_effector) ──
    double ee_x = 0.0, ee_y = 0.0, ee_z = 0.0;
    double ee_roll = 0.0, ee_pitch = 0.0, ee_yaw = 0.0;

    // ── step execution state ─────────────────────────────────────────────────
    bool pose_initialized = false;  // true after first pose callback
    bool action_pending   = false;  // a new action has been latched
    bool executing        = false;  // currently interpolating a step

    // start / target position for the current step (EE frame, base coords)
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
    // The incoming probe-frame action is silently clamped to these
    // magnitudes (direction preserved) before any frame conversion, so the
    // limit means the same thing to the operator regardless of the current
    // calibration rotation or robot orientation. Kept as conservative as
    // ee_incremental_impedance_node.cpp's since this node has not yet been
    // validated on hardware.
    double max_linear_step  = 0.25;  // [m]
    double max_angular_step = 2.0;   // [rad]

    // ── z-axis force control state (rosparam-overridable) ──────────────────────
    bool   force_control_enabled = false; // gated by ~enable_force_control service
    bool   force_fault           = false; // set by an auto-disable safety trip
    bool   force_feedback_received = false;
    double measured_force_z = 0.0;
    rclcpp::Time last_force_msg_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_force_tick_time{0, 0, RCL_ROS_TIME};        // for dt in the pose-callback-driven loop
    double force_integral = 0.0;
    // cumulative position trim along the EE's (== probe's, see file header)
    // evolving local z-axis, expressed in the base frame -- persists
    // (frozen) across enable/disable so the physically-achieved contact
    // indentation is never snapped back, only actively adjusted while enabled
    Eigen::Vector3d force_trim_offset = Eigen::Vector3d::Zero();

    // Target/gains -- MUST be verified on the real sensor/mechanism before use.
    double target_force_z      = 2.0;    // [N] fixed aim value, probe-frame z-axis
    double force_sign          = -1.0;    // flip to +-1 to correct loop direction w/o rebuilding
    // double force_kp            = 0.0005; // [ (m/s) / N ]
    double force_kp            = 0.08; // [ (m/s) / N ]
    double force_ki            = 0.0;    // [ (m/s) / (N*s) ]
    double max_force_step_rate = 0.015;  // [m/s] velocity cap on the trim
    double max_force_travel    = 0.05;   // [m] hard cap on cumulative trim norm
    double max_allowed_force_z = 8.0;    // [N] hard safety cutoff on |measured force|
    double force_feedback_timeout = 0.5; // [s] treat feedback as stale beyond this
    double stiffness_z_active  = 300.0;  // k_fri.z while force control is enabled (compliant)

public:
    rclcpp::Node::SharedPtr node() { return nh; }  // ROS 2: node to spin in main()

    ProbeIncrementalImpedanceNode() {
        double probe_length, probe_y_offset, probe_z_rotation_deg;
        probe_length = nh->declare_parameter<double>("probe_length", 0.252);
        probe_y_offset = nh->declare_parameter<double>("probe_y_offset", 0.00775);
        probe_z_rotation_deg = nh->declare_parameter<double>("probe_z_rotation_deg", 5.0);

        double theta = probe_z_rotation_deg * M_PI / 180.0;
        R_E_P = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        p_E_P = R_E_P * Eigen::Vector3d(0.0, probe_y_offset, 0.0)
                + Eigen::Vector3d(0.0, 0.0, probe_length);
        R_P_E = R_E_P.transpose();
        p_P_E = -R_P_E * p_E_P;

        RCLCPP_INFO(nh->get_logger(), "[probe_incremental_impedance_node] Calibration: probe_length=%.5f m, "
                 "probe_y_offset=%.5f m, probe_z_rotation_deg=%.3f deg",
                 probe_length, probe_y_offset, probe_z_rotation_deg);

        string pose_topic, action_topic, command_topic, force_topic;
        pose_topic = nh->declare_parameter<string>("pose_topic", "/cartesian_position/pose/end_effector");
        action_topic = nh->declare_parameter<string>("action_topic", "/probe_action_impedance");
        command_topic = nh->declare_parameter<string>("command_topic", "/lwr/cartesian_impedance_controller/command");
        force_topic = nh->declare_parameter<string>("force_topic", "/Force_calibrated");

        T = nh->declare_parameter("step_duration",     T);
        stiffness_linear = nh->declare_parameter("stiffness_linear",  stiffness_linear);
        stiffness_angular = nh->declare_parameter("stiffness_angular", stiffness_angular);
        damping = nh->declare_parameter("damping",           damping);
        max_linear_step = nh->declare_parameter("max_linear_step",   max_linear_step);
        max_angular_step = nh->declare_parameter("max_angular_step",  max_angular_step);

        target_force_z = nh->declare_parameter("target_force_z",         target_force_z);
        force_sign = nh->declare_parameter("force_sign",             force_sign);
        force_kp = nh->declare_parameter("force_kp",               force_kp);
        force_ki = nh->declare_parameter("force_ki",               force_ki);
        max_force_step_rate = nh->declare_parameter("max_force_step_rate",    max_force_step_rate);
        max_force_travel = nh->declare_parameter("max_force_travel",       max_force_travel);
        max_allowed_force_z = nh->declare_parameter("max_allowed_force_z",    max_allowed_force_z);
        force_feedback_timeout = nh->declare_parameter("force_feedback_timeout", force_feedback_timeout);
        stiffness_z_active = nh->declare_parameter("stiffness_z_active",     stiffness_z_active);

        sub_pose   = nh->create_subscription<lwr_controllers::msg::PoseRPY>(pose_topic, 10,
                                  std::bind(&ProbeIncrementalImpedanceNode::callback_pose, this, std::placeholders::_1));
        sub_action = nh->create_subscription<geometry_msgs::msg::Twist>(action_topic, 1,
                                  std::bind(&ProbeIncrementalImpedanceNode::callback_action, this, std::placeholders::_1));
        sub_force  = nh->create_subscription<geometry_msgs::msg::Wrench>(force_topic, 10,
                                  std::bind(&ProbeIncrementalImpedanceNode::callback_force, this, std::placeholders::_1));

        srv_enable_force = nh->create_service<std_srvs::srv::SetBool>("~/enable_force_control",
                                  [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                         std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                                      handle_enable_force_control(*req, *res);});

        pub_command = nh->create_publisher<lwr_controllers::msg::CartesianImpedancePoint>(command_topic, 10);

        RCLCPP_INFO(nh->get_logger(), "[probe_incremental_impedance_node] Ready. pose_topic=%s action_topic=%s "
                 "command_topic=%s force_topic=%s. Force control DISABLED at startup -- "
                 "call ~enable_force_control to arm it. Waiting for first pose...",
                 pose_topic.c_str(), action_topic.c_str(), command_topic.c_str(), force_topic.c_str());
    }

    // ── force feedback callback ─────────────────────────────────────────────────
    void callback_force(const geometry_msgs::msg::Wrench::ConstSharedPtr force_msg) {
        // Same convention as ee_incremental_impedance_node.cpp: force.z read
        // directly, no rotation applied.
        measured_force_z = force_msg->force.z;
        last_force_msg_time = nh->now();
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
                (nh->now() - last_force_msg_time).seconds() > force_feedback_timeout) {
                res.success = false;
                res.message = "no recent force feedback -- refusing to enable";
                return true;
            }
            force_fault           = false;
            force_integral         = 0.0;
            force_control_enabled = true;
            res.success = true;
            res.message = "force control enabled";
            RCLCPP_WARN(nh->get_logger(), "[probe_incremental_impedance_node] Force control ENABLED. "
                     "target_force_z=%.3f N force_sign=%.0f", target_force_z, force_sign);
        } else {
            force_control_enabled = false;
            force_fault            = false;
            res.success = true;
            res.message = "force control disabled";
            RCLCPP_WARN(nh->get_logger(), "[probe_incremental_impedance_node] Force control DISABLED. "
                     "Trim held at current value (no snap-back).");
        }
        return true;
    }

    // ── action callback ───────────────────────────────────────────────────────
    void callback_action(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!pose_initialized) {
            RCLCPP_WARN(nh->get_logger(), "[probe_incremental_impedance_node] Action received before pose is "
                     "initialised — ignoring.");
            return;
        }
        if (executing) {
            RCLCPP_WARN(nh->get_logger(), "[probe_incremental_impedance_node] Still executing previous step — "
                     "ignoring new action.");
            return;
        }

        // ── All 6-DoF deltas are expressed in the PROBE's own frame ───────────
        // ee_x/ee_y/ee_z is the MEASURED/actual EE pose, which already
        // reflects the physical effect of any active force_trim_offset (the
        // z-force loop continuously nudges the real robot). force_trim_offset
        // is layered on top exactly once, at publish time, in callback_pose().
        // Subtract it back out here so the new step's start/target stay in
        // that same trim-free frame -- otherwise the trim gets folded into
        // xs/xt implicitly (via ee_x) AND re-added explicitly at publish
        // time, producing a step discontinuity (a visible "fall") the
        // instant a new action starts.
        double base_ee_x = ee_x - force_trim_offset(0);
        double base_ee_y = ee_y - force_trim_offset(1);
        double base_ee_z = ee_z - force_trim_offset(2);

        xs = base_ee_x;
        ys = base_ee_y;
        zs = base_ee_z;

        // Current EE rotation matrix (ZYX: R = Rz * Ry * Rx, matches KDL RPY)
        Eigen::Matrix3d R_cur =
            (Eigen::AngleAxisd(ee_yaw,   Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(ee_pitch, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(ee_roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
        Eigen::Vector3d p_cur(base_ee_x, base_ee_y, base_ee_z);

        q_start = Eigen::Quaterniond(R_cur).normalized();

        // ── Safety clamping (in the probe frame, before any conversion) ───────
        Eigen::Vector3d dp_local(msg->linear.x, msg->linear.y, msg->linear.z);
        {
            double lin_norm = dp_local.norm();
            if (lin_norm > max_linear_step) {
                dp_local *= max_linear_step / lin_norm;
                RCLCPP_WARN(nh->get_logger(), "[probe_incremental_impedance_node] Linear step clamped: "
                         "%.4f m -> %.4f m (max_linear_step)", lin_norm, max_linear_step);
            }
        }

        Eigen::Vector3d da(msg->angular.x, msg->angular.y, msg->angular.z);
        {
            double ang_norm = da.norm();
            if (ang_norm > max_angular_step) {
                da *= max_angular_step / ang_norm;
                RCLCPP_WARN(nh->get_logger(), "[probe_incremental_impedance_node] Angular step clamped: "
                         "%.4f rad -> %.4f rad (max_angular_step)", ang_norm, max_angular_step);
            }
        }

        if (force_control_enabled && std::abs(dp_local(2)) > 1e-9) {
            RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 2000, "[probe_incremental_impedance_node] Force control is "
                     "active -- ignoring the z component of this action (dz=%.4f). "
                     "z is governed by the force loop, not by ~action_topic, while enabled.",
                     dp_local(2));
            dp_local(2) = 0.0;
        }
        // z is always still position-controlled while force control is disabled,
        // and always computed here regardless (force_trim_offset is added on top
        // at publish time, not folded into xt/yt/zt) -- see callback_pose().

        // ── Current probe pose in base frame: T_B_P = T_B_E * T_E_P ───────────
        Eigen::Matrix3d R_B_P_cur = R_cur * R_E_P;
        Eigen::Vector3d p_B_P_cur = p_cur + R_cur * p_E_P;

        // ── Delta transform, expressed in the probe's own frame ───────────────
        Eigen::Matrix3d R_delta =
            (Eigen::AngleAxisd(da(2), Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(da(1), Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(da(0), Eigen::Vector3d::UnitX())).toRotationMatrix();

        // ── Target probe pose: delta applied in probe's own current frame ─────
        Eigen::Matrix3d R_B_P_tgt = R_B_P_cur * R_delta;
        Eigen::Vector3d p_B_P_tgt = p_B_P_cur + R_B_P_cur * dp_local;

        // ── Convert target probe pose back to an EE target: T_B_E = T_B_P * T_E_P^-1 ──
        Eigen::Matrix3d R_B_E_tgt = R_B_P_tgt * R_P_E;
        Eigen::Vector3d p_B_E_tgt = p_B_P_tgt + R_B_P_tgt * p_P_E;

        xt = p_B_E_tgt(0);
        yt = p_B_E_tgt(1);
        zt = p_B_E_tgt(2);

        q_target = Eigen::Quaterniond(R_B_E_tgt).normalized();
        // Take the shorter arc: q and -q represent the same rotation, but
        // slerp between them is not equivalent — pick whichever target sign
        // is nearest q_start so we never spin the "long way around".
        if (q_start.dot(q_target) < 0.0) {
            q_target.coeffs() *= -1.0;
        }

        action_pending = true;

        RCLCPP_INFO(nh->get_logger(), "[probe_incremental_impedance_node] New probe-frame action received "
                 "(after clamping): delta dx=%.4f dy=%.4f dz=%.4f droll=%.4f dpitch=%.4f "
                 "dyaw=%.4f -> EE target x=%.4f y=%.4f z=%.4f",
                 dp_local(0), dp_local(1), dp_local(2), da(0), da(1), da(2), xt, yt, zt);
    }

    // ── z-axis force control update ─────────────────────────────────────────────
    // Admittance-style outer loop: trims force_trim_offset (base-frame 3D vector)
    // along the EE's (== probe's) *current* local z-axis so measured_force_z
    // converges to target_force_z. Runs once per pose-callback tick.
    // Auto-disables on any safety trip.
    void update_force_control(const rclcpp::Time& now, const Eigen::Matrix3d& R_now) {
        if (!force_control_enabled) {
            return;
        }

        if (!force_feedback_received ||
            (now - last_force_msg_time).seconds() > force_feedback_timeout) {
            RCLCPP_ERROR(nh->get_logger(), "[probe_incremental_impedance_node] Force feedback stale (>%.2f s) -- "
                     "disabling force control.", force_feedback_timeout);
            force_control_enabled = false;
            force_fault            = true;
            return;
        }

        if (std::abs(measured_force_z) > max_allowed_force_z) {
            RCLCPP_ERROR(nh->get_logger(), "[probe_incremental_impedance_node] |measured F_z|=%.2f N exceeds "
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

        Eigen::Vector3d ez_world = R_now.col(2);  // EE's (== probe's) current local z-axis, in base frame
        Eigen::Vector3d candidate = force_trim_offset + dz_local * ez_world;
        if (candidate.norm() > max_force_travel) {
            candidate = candidate.normalized() * max_force_travel;
            RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 1000, "[probe_incremental_impedance_node] max_force_travel "
                     "(%.4f m) reached -- holding trim, not tracking further.",
                     max_force_travel);
        }
        force_trim_offset = candidate;
    }

    // ── pose callback ────────────────────────────────────────────────────────
    void callback_pose(const lwr_controllers::msg::PoseRPY::ConstSharedPtr pose_msg) {

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
            RCLCPP_INFO(nh->get_logger(), "[probe_incremental_impedance_node] Pose initialised. "
                     "Waiting for action...");
        }

        // ── start executing when a new action has been latched ────────────────
        if (action_pending) {
            action_pending  = false;
            executing       = true;
            step_start_time = nh->now();
            RCLCPP_INFO(nh->get_logger(), "[probe_incremental_impedance_node] Executing step (T=%.2f s) ...", T);
        }

        // ── base x/y/z/orientation target from the position-control step ──────
        double base_x, base_y, base_z;
        Eigen::Quaterniond q_interp;

        if (executing) {
            double elapsed = (nh->now() - step_start_time).seconds();
            double Rho     = elapsed / T;
            bool   finished = Rho >= 1.0;
            Rho = std::min(std::max(Rho, 0.0), 1.0);

            // Same 5th-order polynomial timing law used elsewhere in this
            // workspace, parameterised by elapsed-time fraction Rho and
            // shared by both position and orientation slerp.
            double Sigma = 10.0*std::pow(Rho,3) - 15.0*std::pow(Rho,4) + 6.0*std::pow(Rho,5);

            base_x = xs + (xt - xs) * Sigma;
            base_y = ys + (yt - ys) * Sigma;
            base_z = zs + (zt - zs) * Sigma;
            q_interp = q_start.slerp(Sigma, q_target);

            if (finished) {
                executing = false;
                RCLCPP_INFO(nh->get_logger(), "[probe_incremental_impedance_node] Step completed. "
                         "Holding target pose. Waiting for next action...");
            }
        } else {
            // Hold current target pose (no motion)
            base_x = xt; base_y = yt; base_z = zt;
            q_interp = q_target;
        }

        // ── z-axis force control: continuously trims a base-frame offset along
        //    the EE's (== probe's) current local z-axis, added on top of the
        //    position target above. Independent of the executing/holding state
        //    -- it runs every tick so it can hold contact force even while
        //    otherwise idle. ─────────────────────────────────────────────────
        Eigen::Matrix3d R_now =
            (Eigen::AngleAxisd(ee_yaw,   Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(ee_pitch, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(ee_roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
        update_force_control(nh->now(), R_now);

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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    ProbeIncrementalImpedanceNode node;
    rclcpp::spin(node.node());
    rclcpp::shutdown();
    return 0;
}
