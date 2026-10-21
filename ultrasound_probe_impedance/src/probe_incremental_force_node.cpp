/*
 * probe_incremental_force_node.cpp
 *
 * Same framework as probe_incremental_impedance_node.cpp: drives the KUKA LWR
 * Cartesian impedance controller from a relative 6-DoF delta expressed in the
 * ULTRASOUND PROBE's own frame:
 *   linear.x/y/z  -> dx, dy, dz  [metres]
 *   angular.x/y/z -> droll, dpitch, dyaw  [radians]
 *
 * The probe frame is rigidly attached to the end-effector (flange) frame via
 * a fixed calibration transform T_E_P (flange E -> probe P), loaded from
 * YAML with the same convention as probe_incremental_impedance_node.cpp:
 *
 *   T_E_P = Trans_z(probe_length) * Rot_z(probe_z_rotation_deg) * Trans_y(probe_y_offset)
 *
 * On receipt of an action this node:
 *   1. computes the current probe pose in the base frame from the current
 *      EE pose feedback and T_E_P,
 *   2. applies the requested delta in the probe's own current frame,
 *   3. converts the resulting target probe pose back to an EE target via
 *      T_E_P^-1,
 *   4. executes one smooth (5th-order polynomial + quaternion SLERP) step
 *      from the current EE pose to that EE target, then holds it, publishing
 *      an absolute base-frame lwr_controllers/CartesianImpedancePoint.
 * Normal and recovery topics share this executor; only the latched trajectory
 * duration differs.
 *
 * Topic in  : ~action_topic  (default /probe_action_force, geometry_msgs/Twist)
 * Topic in  : ~recovery_action_topic (default /probe_action_force_recovery,
 *             geometry_msgs/Twist; uses ~recovery_step_duration)
 * Topic in  : ~pose_topic    (default /cartesian_position/pose/end_effector, lwr_controllers/PoseRPY)
 * Topic out : ~command_topic (default /lwr/cartesian_impedance_controller/command, lwr_controllers/CartesianImpedancePoint)
 *
 * NOTE: ~action_topic defaults to a topic distinct from /probe_action_impedance
 * (used by probe_incremental_impedance_node.cpp), /ee_action_impedance (used by
 * ultrasound_trajectory's ee_incremental_impedance_node.cpp) and /probe_action
 * (used by probe_relative_motion_node.cpp) on purpose -- those nodes must never
 * be cross-triggered by a command meant for this one.
 *
 * SAFETY: ~command_topic is shared with probe_incremental_impedance_node.cpp
 * and ee_incremental_impedance_node.cpp. Do not run this node at the same time
 * as either of those unless ~command_topic has been remapped so exactly one
 * node is driving the impedance controller.
 *
 * ── z-axis force control (probe/EE frame) ───────────────────────────────────
 * x and y (scanning motion) and orientation are ALWAYS position-controlled
 * from ~action_topic, at all times, including while force control is active
 * -- only z is ever handed over to the force loop. This is what makes
 * "scan while holding a constant contact force" possible: the two run
 * concurrently and independently, x/y/orientation via the normal
 * trajectory-interpolated x_fri.position/orientation, z via a continuously
 * running closed-loop feedforward force command in f_fri.force.z.
 *
 * Distinct from probe_incremental_impedance_node.cpp's closed-loop admittance
 * trim on x_fri.position, the mechanism here still follows
 * ultrasound_trajectory/linear_traj_node_Luca.cpp's original idea of driving
 * the feedforward force channel (f_fri.force.z) directly rather than trimming
 * position -- but Luca's node only ever needed a single open-loop ramp for
 * one static contact event, so that is not reused as-is here. Instead,
 * update_force_control() runs every pose-callback tick while ~threshold_f is
 * crossed (contact detected) and continuously integrates the force error
 * (target_force_z - |measured_force_z|) into the feedforward reference via
 * ~force_kp, so it converges to and HOLDS ~target_force_z rather than ramping
 * once to a cap and stopping. If contact is lost (measured force rises back
 * above ~threshold_f, e.g. the probe lifts off), the reference resets to 0
 * and re-ramps from scratch the next time contact is detected -- this can
 * happen any number of times while armed, with no need to re-call the enable
 * service. ~max_force_reference remains a hard safety cap on the feedforward
 * command itself, independent of the control loop, in case of a sensor fault
 * or an unexpectedly compliant surface.
 *
 * NOTE: this node reads ~force_topic default /Force_calibrated
 * (geometry_msgs/Wrench), the same ANN gravity/tool-weight-compensated
 * signal probe_incremental_impedance_node.cpp uses -- published by
 * sriforcesensor/src/ANN/tool_comp_ros.py from raw /Force plus current EE
 * orientation -- rather than Luca's raw /Force (geometry_msgs/Twist) with a
 * single fixed offset, which only cancels tool weight correctly at whatever
 * one orientation it was tuned for. This node moves the probe through
 * arbitrary orientations during scanning, so the orientation-aware
 * calibrated signal is required.
 *
 * SAFETY: force control starts DISABLED and must be explicitly turned on via
 * the ~enable_force_control (std_srvs/SetBool) service. It refuses to arm
 * without a live pose and recent force feedback, and auto-disables if force
 * feedback goes stale while running. Verify ~threshold_f, ~force_kp,
 * ~target_force_z and ~max_force_reference against your actual sensor before
 * relying on this for real tissue contact.
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
#include <stdexcept>

using namespace std;

class ProbeIncrementalForceNode {

private:
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("probe_incremental_force_node");
    rclcpp::SubscriptionBase::SharedPtr sub_pose;    // current EE pose
    rclcpp::SubscriptionBase::SharedPtr sub_action;  // normal incoming 6-DoF probe-frame action
    rclcpp::SubscriptionBase::SharedPtr sub_recovery_action; // slower pose-recovery action
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

    // start/target orientation, interpolated via slerp
    Eigen::Quaterniond q_start, q_target;

    // wall-clock time the current step began
    rclcpp::Time step_start_time{0, 0, RCL_ROS_TIME};

    // ── trajectory / controller parameters (rosparam-overridable) ─────────────
    double T = 3.0;                 // normal step duration [s], real elapsed time
    double recovery_T = 3.0;        // recovery step duration [s]
    double active_T = 3.0;          // duration latched for the accepted action
    string active_profile = "normal";
    double stiffness_linear  = 1500.0;
    double stiffness_angular = 250.0;
    double damping           = 0.8;

    // ── safety limits (rosparam-overridable) ───────────────────────────────────
    double max_linear_step  = 0.25;  // [m]
    double max_angular_step = 2.0;   // [rad]

    // ── z-axis force control state: continuous closed loop on the feedforward
    //    channel, contact-gated. See the file header note. ───────────────────
    enum ControlState {
        IMPEDANCE_CONTROL,  // not in contact (or disabled) -- f_fri.force.z = 0
        FORCE_CONTROL       // in contact -- integrating toward/holding target_force_z
    };
    ControlState state_control = IMPEDANCE_CONTROL;

    bool   force_control_enabled   = false; // gated by ~enable_force_control service
    bool   force_feedback_received = false;
    double measured_force_z = 0.0;
    rclcpp::Time last_force_msg_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_force_tick_time{0, 0, RCL_ROS_TIME};         // for dt in the pose-callback-driven loop

    double new_force_reference = 0.0;       // continuously-held feedforward force command [N]

    // Gains/limits -- MUST be verified on the real sensor/mechanism before use.
    double threshold_f         = -0.1;   // [N] measured_force_z below this = "in contact"
    double force_kp            = 0.5;    // [1/s] integral gain on (target - |measured|)
    double target_force_z      = 5.0;    // [N] |measured_force_z| to converge to and hold
    double max_force_reference = 8.0;    // [N] hard safety cap on the feedforward command
    double force_feedback_timeout = 0.5; // [s] treat feedback as stale beyond this

public:
    rclcpp::Node::SharedPtr node() { return nh; }  // ROS 2: node to spin in main()

    ProbeIncrementalForceNode() {
        double probe_length, probe_y_offset, probe_z_rotation_deg;
        probe_length = nh->declare_parameter<double>("probe_length", 0.252);
        probe_y_offset = nh->declare_parameter<double>("probe_y_offset", 0.00775);
        probe_z_rotation_deg = nh->declare_parameter<double>("probe_z_rotation_deg", 0.0);

        double theta = probe_z_rotation_deg * M_PI / 180.0;
        R_E_P = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        p_E_P = R_E_P * Eigen::Vector3d(0.0, probe_y_offset, 0.0)
                + Eigen::Vector3d(0.0, 0.0, probe_length);
        R_P_E = R_E_P.transpose();
        p_P_E = -R_P_E * p_E_P;

        RCLCPP_INFO(nh->get_logger(), "[probe_incremental_force_node] Calibration: probe_length=%.5f m, "
                 "probe_y_offset=%.5f m, probe_z_rotation_deg=%.3f deg",
                 probe_length, probe_y_offset, probe_z_rotation_deg);

        string pose_topic, action_topic, recovery_action_topic, command_topic, force_topic;
        pose_topic = nh->declare_parameter<string>("pose_topic", "/cartesian_position/pose/end_effector");
        action_topic = nh->declare_parameter<string>("action_topic", "/probe_action_force");
        recovery_action_topic = nh->declare_parameter<string>("recovery_action_topic", "/probe_action_force_recovery");
        command_topic = nh->declare_parameter<string>("command_topic", "/lwr/cartesian_impedance_controller/command");
        force_topic = nh->declare_parameter<string>("force_topic", "/Force_calibrated");

        T = nh->declare_parameter("step_duration",     T);
        recovery_T = nh->declare_parameter("recovery_step_duration", recovery_T);
        if (!(std::isfinite(T) && T > 0.0 &&
              std::isfinite(recovery_T) && recovery_T > 0.0)) {
            throw std::runtime_error("step_duration and recovery_step_duration must be finite and positive");
        }
        stiffness_linear = nh->declare_parameter("stiffness_linear",  stiffness_linear);
        stiffness_angular = nh->declare_parameter("stiffness_angular", stiffness_angular);
        damping = nh->declare_parameter("damping",           damping);
        max_linear_step = nh->declare_parameter("max_linear_step",   max_linear_step);
        max_angular_step = nh->declare_parameter("max_angular_step",  max_angular_step);

        threshold_f = nh->declare_parameter("threshold_f",            threshold_f);
        force_kp = nh->declare_parameter("force_kp",               force_kp);
        target_force_z = nh->declare_parameter("target_force_z",         target_force_z);
        max_force_reference = nh->declare_parameter("max_force_reference",    max_force_reference);
        force_feedback_timeout = nh->declare_parameter("force_feedback_timeout", force_feedback_timeout);

        sub_pose   = nh->create_subscription<lwr_controllers::msg::PoseRPY>(pose_topic, 10,
                                  std::bind(&ProbeIncrementalForceNode::callback_pose, this, std::placeholders::_1));
        sub_action = nh->create_subscription<geometry_msgs::msg::Twist>(action_topic, 1,
                                  std::bind(&ProbeIncrementalForceNode::callback_action, this, std::placeholders::_1));
        sub_recovery_action = nh->create_subscription<geometry_msgs::msg::Twist>(recovery_action_topic, 1,
                                  std::bind(&ProbeIncrementalForceNode::callback_recovery_action, this, std::placeholders::_1));
        sub_force  = nh->create_subscription<geometry_msgs::msg::Wrench>(force_topic, 10,
                                  std::bind(&ProbeIncrementalForceNode::callback_force, this, std::placeholders::_1));

        srv_enable_force = nh->create_service<std_srvs::srv::SetBool>("~/enable_force_control",
                                  [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                         std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                                      handle_enable_force_control(*req, *res);});

        pub_command = nh->create_publisher<lwr_controllers::msg::CartesianImpedancePoint>(command_topic, 10);

        RCLCPP_INFO(nh->get_logger(), "[probe_incremental_force_node] Ready. pose_topic=%s action_topic=%s recovery_action_topic=%s "
                 "command_topic=%s force_topic=%s. Force control DISABLED at startup -- "
                 "call ~enable_force_control to arm it. Waiting for first pose...",
                 pose_topic.c_str(), action_topic.c_str(), recovery_action_topic.c_str(),
                 command_topic.c_str(), force_topic.c_str());
    }

    // ── force feedback callback ─────────────────────────────────────────────────
    // Just latches the latest reading; the control law itself runs from
    // callback_pose() via update_force_control(), decoupled from whatever
    // rate ~force_topic actually publishes at (matches the pattern
    // probe_incremental_impedance_node.cpp uses for its own force loop).
    void callback_force(const geometry_msgs::msg::Wrench::ConstSharedPtr force_msg) {
        measured_force_z = force_msg->force.z;
        force_feedback_received = true;
        last_force_msg_time = nh->now();
    }

    // ── z-axis force control update ─────────────────────────────────────────────
    // Runs once per pose-callback tick. While ~force_control_enabled and in
    // contact (measured_force_z < threshold_f), continuously integrates the
    // force error into new_force_reference so it converges to and HOLDS
    // target_force_z -- this is what lets the node keep a stable contact
    // force through an arbitrarily long scan, not just a single push. On
    // contact loss (or if disabled/stale feedback), the reference resets to
    // 0 so the next contact ramps up cleanly instead of slamming in whatever
    // value was last held.
    void update_force_control(const rclcpp::Time& now) {
        if (!force_control_enabled) {
            state_control = IMPEDANCE_CONTROL;
            new_force_reference = 0.0;
            return;
        }

        if (!force_feedback_received ||
            (now - last_force_msg_time).seconds() > force_feedback_timeout) {
            RCLCPP_ERROR_THROTTLE(nh->get_logger(), *nh->get_clock(), 1000, "[probe_incremental_force_node] Force feedback stale "
                     "(>%.2f s) -- disabling force control.", force_feedback_timeout);
            force_control_enabled = false;
            state_control = IMPEDANCE_CONTROL;
            new_force_reference = 0.0;
            return;
        }

        bool in_contact = measured_force_z < threshold_f;
        if (!in_contact) {
            if (state_control == FORCE_CONTROL) {
                RCLCPP_INFO(nh->get_logger(), "[probe_incremental_force_node] Contact lost "
                         "(measured_force_z=%.3f N >= threshold_f=%.3f N) -- "
                         "back to IMPEDANCE_CONTROL.", measured_force_z, threshold_f);
            }
            state_control = IMPEDANCE_CONTROL;
            new_force_reference = 0.0;
            return;
        }

        if (state_control != FORCE_CONTROL) {
            state_control = FORCE_CONTROL;
            last_force_tick_time = now;  // reset dt baseline for this contact episode
            RCLCPP_INFO(nh->get_logger(), "[probe_incremental_force_node] Contact detected "
                     "(measured_force_z=%.3f N < threshold_f=%.3f N) -- FORCE_CONTROL "
                     "engaged, converging toward target_force_z=%.3f N",
                     measured_force_z, threshold_f, target_force_z);
        }

        double dt = (last_force_tick_time.nanoseconds() == 0) ? 0.0 : (now - last_force_tick_time).seconds();
        last_force_tick_time = now;
        if (dt <= 0.0 || dt > 0.5) {
            return;  // first tick of this contact episode, or a clock jump
        }

        double error = target_force_z - std::abs(measured_force_z);
        new_force_reference += force_kp * error * dt;
        new_force_reference = std::min(std::max(new_force_reference, 0.0), max_force_reference);

        RCLCPP_INFO_THROTTLE(nh->get_logger(), *nh->get_clock(), 500, "[probe_incremental_force_node] FORCE CONTROL: "
                           "reference=%.3f N measured=%.3f N target=%.3f N",
                           new_force_reference, measured_force_z, target_force_z);
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
            new_force_reference    = 0.0;
            state_control          = IMPEDANCE_CONTROL;
            force_control_enabled  = true;
            res.success = true;
            res.message = "force control enabled (armed, waiting for contact)";
            RCLCPP_WARN(nh->get_logger(), "[probe_incremental_force_node] Force control ENABLED. "
                     "threshold_f=%.3f N force_kp=%.3f target_force_z=%.3f N "
                     "max_force_reference=%.3f N",
                     threshold_f, force_kp, target_force_z, max_force_reference);
        } else {
            force_control_enabled = false;
            res.success = true;
            res.message = "force control disabled";
            RCLCPP_WARN(nh->get_logger(), "[probe_incremental_force_node] Force control DISABLED.");
        }
        return true;
    }

    // ── action callback ───────────────────────────────────────────────────────
    void callback_action(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        accept_action(msg, T, "normal");
    }

    void callback_recovery_action(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        accept_action(msg, recovery_T, "recovery");
    }

    void accept_action(const geometry_msgs::msg::Twist::ConstSharedPtr msg,
                       double duration, const string& profile) {
        if (!pose_initialized) {
            RCLCPP_WARN(nh->get_logger(), "[probe_incremental_force_node] Action received before pose is "
                     "initialised — ignoring.");
            return;
        }
        if (executing || action_pending) {
            RCLCPP_WARN(nh->get_logger(), "[probe_incremental_force_node] Previous step is executing or pending — "
                     "ignoring new %s action.", profile.c_str());
            return;
        }

        // ── All 6-DoF deltas are expressed in the PROBE's own frame ───────────
        double base_ee_x = ee_x;
        double base_ee_y = ee_y;
        double base_ee_z = ee_z;

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
                RCLCPP_WARN(nh->get_logger(), "[probe_incremental_force_node] Linear step clamped: "
                         "%.4f m -> %.4f m (max_linear_step)", lin_norm, max_linear_step);
            }
        }

        Eigen::Vector3d da(msg->angular.x, msg->angular.y, msg->angular.z);
        {
            double ang_norm = da.norm();
            if (ang_norm > max_angular_step) {
                da *= max_angular_step / ang_norm;
                RCLCPP_WARN(nh->get_logger(), "[probe_incremental_force_node] Angular step clamped: "
                         "%.4f rad -> %.4f rad (max_angular_step)", ang_norm, max_angular_step);
            }
        }

        if (force_control_enabled && std::abs(dp_local(2)) > 1e-9) {
            RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 2000, "[probe_incremental_force_node] Force control is "
                     "active -- ignoring the z component of this action (dz=%.4f). "
                     "z is governed by the force loop, not by ~action_topic, while enabled; "
                     "x/y/orientation are unaffected and continue to be action-driven.",
                     dp_local(2));
            dp_local(2) = 0.0;
        }

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
        if (q_start.dot(q_target) < 0.0) {
            q_target.coeffs() *= -1.0;
        }

        action_pending = true;
        active_T = duration;
        active_profile = profile;

        RCLCPP_INFO(nh->get_logger(), "[probe_incremental_force_node] New probe-frame action received "
                 "(after clamping): delta dx=%.4f dy=%.4f dz=%.4f droll=%.4f dpitch=%.4f "
                 "dyaw=%.4f profile=%s duration=%.2f s -> EE target x=%.4f y=%.4f z=%.4f",
                 dp_local(0), dp_local(1), dp_local(2), da(0), da(1), da(2),
                 active_profile.c_str(), active_T, xt, yt, zt);
    }

    // ── pose callback ────────────────────────────────────────────────────────
    void callback_pose(const lwr_controllers::msg::PoseRPY::ConstSharedPtr pose_msg) {

        ee_x     = pose_msg->position.x;
        ee_y     = pose_msg->position.y;
        ee_z     = pose_msg->position.z;
        ee_roll  = pose_msg->orientation.roll;
        ee_pitch = pose_msg->orientation.pitch;
        ee_yaw   = pose_msg->orientation.yaw;

        Eigen::Matrix3d R_now =
            (Eigen::AngleAxisd(ee_yaw,   Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(ee_pitch, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(ee_roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();
        Eigen::Quaterniond q_now = Eigen::Quaterniond(R_now).normalized();

        // ── initialise command_msg once with the robot's starting pose,
        //    stiffness, damping and zero force/torque ──────────────────────────
        if (!pose_initialized) {
            command_msg.header.frame_id = frame_id;
            command_msg.x_fri.position.x = ee_x;
            command_msg.x_fri.position.y = ee_y;
            command_msg.x_fri.position.z = ee_z;
            command_msg.x_fri.orientation.x = q_now.x();
            command_msg.x_fri.orientation.y = q_now.y();
            command_msg.x_fri.orientation.z = q_now.z();
            command_msg.x_fri.orientation.w = q_now.w();

            command_msg.k_fri.x = stiffness_linear;  command_msg.k_fri.y = stiffness_linear;  command_msg.k_fri.z = stiffness_linear;
            command_msg.k_fri.rx = stiffness_angular; command_msg.k_fri.ry = stiffness_angular; command_msg.k_fri.rz = stiffness_angular;

            command_msg.d_fri.x = damping; command_msg.d_fri.y = damping; command_msg.d_fri.z = damping;
            command_msg.d_fri.rx = damping; command_msg.d_fri.ry = damping; command_msg.d_fri.rz = damping;

            command_msg.f_fri.force.x = 0.0;  command_msg.f_fri.force.y = 0.0;  command_msg.f_fri.force.z = 0.0;
            command_msg.f_fri.torque.x = 0.0; command_msg.f_fri.torque.y = 0.0; command_msg.f_fri.torque.z = 0.0;

            // hold at the current pose until the first action arrives
            xs = xt = ee_x; ys = yt = ee_y; zs = zt = ee_z;
            q_start = q_target = q_now;

            pose_initialized = true;
            RCLCPP_INFO(nh->get_logger(), "[probe_incremental_force_node] Pose initialised. "
                     "Waiting for action...");
        }

        // ── start executing when a new action has been latched ────────────────
        if (action_pending) {
            action_pending  = false;
            executing       = true;
            step_start_time = nh->now();
            RCLCPP_INFO(nh->get_logger(), "[probe_incremental_force_node] Executing %s step (T=%.2f s) ...",
                     active_profile.c_str(), active_T);
        }

        // ── base x/y/z/orientation target from the position-control step ──────
        double base_x, base_y, base_z;
        Eigen::Quaterniond q_interp;

        if (executing) {
            double elapsed = (nh->now() - step_start_time).seconds();
            double Rho     = elapsed / active_T;
            bool   finished = Rho >= 1.0;
            Rho = std::min(std::max(Rho, 0.0), 1.0);

            double Sigma = 10.0*std::pow(Rho,3) - 15.0*std::pow(Rho,4) + 6.0*std::pow(Rho,5);

            base_x = xs + (xt - xs) * Sigma;
            base_y = ys + (yt - ys) * Sigma;
            base_z = zs + (zt - zs) * Sigma;
            q_interp = q_start.slerp(Sigma, q_target);

            if (finished) {
                executing = false;
                RCLCPP_INFO(nh->get_logger(), "[probe_incremental_force_node] Step completed. "
                         "Holding target pose. Waiting for next action...");
            }
        } else {
            // Hold current target pose (no motion)
            base_x = xt; base_y = yt; base_z = zt;
            q_interp = q_target;
        }

        // ── z-axis force control: continuous closed loop, decoupled from the
        //    x/y/orientation trajectory above so action-driven scanning motion
        //    keeps working the whole time force is being held. Runs every tick
        //    regardless of executing/holding state. ─────────────────────────
        update_force_control(nh->now());

        command_msg.x_fri.position.x = base_x;
        command_msg.x_fri.position.y = base_y;
        command_msg.x_fri.position.z = base_z;
        command_msg.x_fri.orientation.x = q_interp.x();
        command_msg.x_fri.orientation.y = q_interp.y();
        command_msg.x_fri.orientation.z = q_interp.z();
        command_msg.x_fri.orientation.w = q_interp.w();

        command_msg.f_fri.force.x = 0.0;
        command_msg.f_fri.force.y = 0.0;
        command_msg.f_fri.force.z = (state_control == FORCE_CONTROL) ? new_force_reference : 0.0;
        command_msg.f_fri.torque.x = 0.0;
        command_msg.f_fri.torque.y = 0.0;
        command_msg.f_fri.torque.z = 0.0;

        pub_command->publish(command_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    ProbeIncrementalForceNode node;
    rclcpp::spin(node.node());
    rclcpp::shutdown();
    return 0;
}
