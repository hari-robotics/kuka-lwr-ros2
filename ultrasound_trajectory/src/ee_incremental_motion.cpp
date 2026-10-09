#include "ultrasound_trajectory/ros2_support.hpp"
#include <functional>
#include <rclcpp/create_timer.hpp>
/*
 * ultrasound_traj_node.cpp
 *
 * Subscribes to /ee_action (geometry_msgs/Twist) for a relative 6-DoF delta:
 *   linear.x/y/z  -> dx, dy, dz  [metres]
 *   angular.x/y/z -> droll, dpitch, dyaw  [radians]
 *
 * On receipt, executes one smooth (5th-order polynomial) step from the
 * current EE pose to (current + delta), then holds the target pose and
 * waits for the next action.
 *
 * Topic in  : /ee_action            (geometry_msgs/Twist)
 * Topic in  : /cartesian_position/pose/end_effector  (lwr_controllers/PoseRPY)
 * Topic out : /lwr/one_task_inverse_kinematics/command (lwr_controllers/PoseRPY)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
//  kuka-lwr Controllers Msgs
#include <lwr_controllers/msg/arm_state.hpp>
#include <lwr_controllers/msg/pose_rpy.hpp>
//  Eigen lib
#include <Eigen/Core>
#include <Eigen/Geometry>   // AngleAxisd, rotation-matrix composition

#include <math.h>
#include <cmath>

using namespace std;

class UltrasoundTrajNode : public rclcpp::Node {

private:
    rclcpp::Subscription<lwr_controllers::msg::PoseRPY>::SharedPtr sub_pose;    // current EE pose
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_action;  // incoming 6-DoF action

    rclcpp::Publisher<lwr_controllers::msg::PoseRPY>::SharedPtr pub_command; // pose command to controller

    lwr_controllers::msg::PoseRPY command_msg;

    // ── current EE state ────────────────────────────────────────────────────
    double ee_x, ee_y, ee_z;
    double ee_roll, ee_pitch, ee_yaw;

    // ── step execution state ─────────────────────────────────────────────────
    bool pose_initialized = false;  // true after first pose callback
    bool action_pending   = false;  // a new action has been received
    bool executing        = false;  // currently interpolating a step

    // start pose for the current step
    double xs, ys, zs;

    // target pose for the current step (rt/pt/yt_ kept for logging only)
    double xt, yt, zt, rt, pt, yt_;

    // start/target orientation, interpolated via slerp (avoids Euler-angle
    // wraparound and gimbal-lock artefacts that independent roll/pitch/yaw
    // interpolation is prone to)
    Eigen::Quaterniond q_start, q_target;

    // wall-clock time the current step began; timing is driven off elapsed
    // time, not a per-callback counter, so it stays correct regardless of the
    // actual /cartesian_position/pose/end_effector publish rate (measured
    // ~200 Hz here, not the 100 Hz previously assumed)
    rclcpp::Time step_start_time{0, 0, RCL_ROS_TIME};

    // ── trajectory parameters ────────────────────────────────────────────────
    double T = 6;  // step duration [seconds] — real elapsed time, not steps

    // ── safety limits ────────────────────────────────────────────────────────
    // Edit these two values to change the per-step safety thresholds.
    // The incoming /ee_action is silently clamped to these magnitudes;
    // direction is preserved.
    static constexpr double MAX_LINEAR_STEP  = 0.25;  // [m]   max translation magnitude per step
    static constexpr double MAX_ANGULAR_STEP = 2;  // [rad] max rotation magnitude per step
    // static constexpr double MAX_LINEAR_STEP  = 0.05;  // [m]   max translation magnitude per step
    // static constexpr double MAX_ANGULAR_STEP = 0.10;  // [rad] max rotation magnitude per step

public:
    UltrasoundTrajNode() : rclcpp::Node("ee_incremental_motion") {
        sub_pose = create_subscription<lwr_controllers::msg::PoseRPY>("/cartesian_position/pose/end_effector", rclcpp::SensorDataQoS(), std::bind(&UltrasoundTrajNode::callback_pose, this, std::placeholders::_1));
        sub_action = create_subscription<geometry_msgs::msg::Twist>("/ee_action", rclcpp::QoS(1), std::bind(&UltrasoundTrajNode::callback_action, this, std::placeholders::_1));

        pub_command = create_publisher<lwr_controllers::msg::PoseRPY>("/lwr/one_task_inverse_kinematics/command", 10);

        RCLCPP_INFO(get_logger(), "[ultrasound_traj_node] Ready. Waiting for first pose...");
    }

    // ── action callback ───────────────────────────────────────────────────────
    void callback_action(const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
        if (!ultrasound_trajectory::finite_twist(*msg)) {return;}
        if (!pose_initialized) {
            RCLCPP_WARN(get_logger(), "[ultrasound_traj_node] Action received before pose is "
                     "initialised — ignoring.");
            return;
        }
        if (executing) {
            RCLCPP_WARN(get_logger(), "[ultrasound_traj_node] Still executing previous step — "
                     "ignoring new action.");
            return;
        }

        // ── All 6-DoF deltas are expressed in the EE's local frame ────────────
        //
        // Both the linear displacement and the angular rotation are given in
        // the end-effector's own coordinate frame, NOT the world frame.
        //
        // Linear: rotate the local displacement into world coordinates first.
        //   p_world_delta = R_current * [dx, dy, dz]ᵀ
        //
        // Angular: post-multiply the orientation (delta applied in EE frame).
        //   R_new = R_current * R_delta
        //
        // ZYX convention (KDL GetRPY / Rotation::RPY):  R = Rz * Ry * Rx

        xs  = ee_x;
        ys  = ee_y;
        zs  = ee_z;

        // Current EE rotation matrix (ZYX: R = Rz * Ry * Rx)
        Eigen::Matrix3d R_cur =
            (Eigen::AngleAxisd(ee_yaw,   Eigen::Vector3d::UnitZ())
           * Eigen::AngleAxisd(ee_pitch, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(ee_roll,  Eigen::Vector3d::UnitX())).toRotationMatrix();

        q_start = Eigen::Quaterniond(R_cur).normalized();

        // ── Safety clamping ───────────────────────────────────────────────────
        // Linear: preserve direction, clamp magnitude to MAX_LINEAR_STEP
        Eigen::Vector3d dp_local(msg->linear.x, msg->linear.y, msg->linear.z);
        {
            double lin_norm = dp_local.norm();
            if (lin_norm > MAX_LINEAR_STEP) {
                dp_local *= MAX_LINEAR_STEP / lin_norm;
                RCLCPP_WARN(get_logger(), "[ultrasound_traj_node] Linear step clamped: "
                         "%.4f m -> %.4f m (MAX_LINEAR_STEP)", lin_norm, MAX_LINEAR_STEP);
            }
        }

        // Angular: preserve direction, clamp magnitude to MAX_ANGULAR_STEP
        Eigen::Vector3d da(msg->angular.x, msg->angular.y, msg->angular.z);
        {
            double ang_norm = da.norm();
            if (ang_norm > MAX_ANGULAR_STEP) {
                da *= MAX_ANGULAR_STEP / ang_norm;
                RCLCPP_WARN(get_logger(), "[ultrasound_traj_node] Angular step clamped: "
                         "%.4f rad -> %.4f rad (MAX_ANGULAR_STEP)", ang_norm, MAX_ANGULAR_STEP);
            }
        }

        // ── Position target: map local displacement to world frame ────────────
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

        // Extract ZYX Euler angles — must match KDL::Rotation::GetRPY()
        // (kept only for the log line below; interpolation uses q_start/q_target)
        yt_ = std::atan2( R_new(1,0),  R_new(0,0));
        pt  = std::atan2(-R_new(2,0),  std::sqrt(R_new(2,1)*R_new(2,1)
                                                + R_new(2,2)*R_new(2,2)));
        rt  = std::atan2( R_new(2,1),  R_new(2,2));

        q_target = Eigen::Quaterniond(R_new).normalized();
        // Take the shorter arc: q and -q represent the same rotation, but
        // slerp between them is not equivalent — pick whichever target sign
        // is nearest q_start so we never spin the "long way around".
        if (q_start.dot(q_target) < 0.0) {
            q_target.coeffs() *= -1.0;
        }

        action_pending = true;

        RCLCPP_INFO(get_logger(), "[ultrasound_traj_node] New action received (after clamping):");
        RCLCPP_INFO(get_logger(), "  delta  : dx=%.4f  dy=%.4f  dz=%.4f  "
                 "droll=%.4f  dpitch=%.4f  dyaw=%.4f",
                 dp_local(0), dp_local(1), dp_local(2),
                 da(0), da(1), da(2));
        RCLCPP_INFO(get_logger(), "  target : x=%.4f  y=%.4f  z=%.4f  "
                 "roll=%.4f  pitch=%.4f  yaw=%.4f",
                 xt, yt, zt, rt, pt, yt_);
    }

    // ── pose callback (runs at whatever rate /cartesian_position/pose/end_effector
    //    publishes, measured ~200 Hz — trajectory timing does not depend on this) ──
    void callback_pose(const lwr_controllers::msg::PoseRPY::ConstSharedPtr& pose_msg) {
        if (!ultrasound_trajectory::finite_pose(*pose_msg)) {return;}

        // Update current EE state
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
            RCLCPP_INFO(get_logger(), "[ultrasound_traj_node] Pose initialised. "
                     "Waiting for /ee_action ...");
        }

        // ── start executing when a new action has been latched ────────────────
        if (action_pending) {
            action_pending  = false;
            executing       = true;
            step_start_time = now();
            RCLCPP_INFO(get_logger(), "[ultrasound_traj_node] Executing step (T=%.2f s) ...", T);
        }

        // ── interpolate and publish ───────────────────────────────────────────
        // Timing is driven by elapsed wall-clock time, not by counting pose
        // callbacks — the previous per-callback counter silently assumed the
        // pose topic publishes at exactly SubSteps (100 Hz); it actually runs
        // at ~200 Hz here, so T no longer mapped to any predictable real
        // duration. This also removes the old in-callback ros::Rate::sleep(),
        // which blocked the subscriber callback itself and caused the queue
        // to back up/drop messages under rate mismatch.
        if (executing) {
            double elapsed = (now() - step_start_time).seconds();
            double Rho      = elapsed / T;
            bool   finished  = Rho >= 1.0;
            Rho = std::min(std::max(Rho, 0.0), 1.0);

            // Same 5th-order polynomial timing law as smooth_traj.h, now
            // parameterised directly by time fraction Rho instead of a step
            // count, and shared by both position and the orientation slerp.
            double Sigma = 10.0*std::pow(Rho,3) - 15.0*std::pow(Rho,4) + 6.0*std::pow(Rho,5);

            command_msg.position.x = xs + (xt - xs) * Sigma;
            command_msg.position.y = ys + (yt - ys) * Sigma;
            command_msg.position.z = zs + (zt - zs) * Sigma;

            // Orientation: slerp along the same timing law, instead of
            // interpolating roll/pitch/yaw independently. This follows the
            // shortest physical rotation arc from q_start to q_target and
            // can't hit the Euler-angle wraparound / gimbal-lock artefacts
            // that made large single-axis rotations (e.g. ~0.75 rad in yaw)
            // occasionally command a wild roll/pitch/yaw path — and the
            // strange downstream joint motion that came with it.
            Eigen::Quaterniond q_interp = q_start.slerp(Sigma, q_target);
            Eigen::Matrix3d R_interp = q_interp.toRotationMatrix();

            // Extract ZYX Euler angles — must match KDL::Rotation::GetRPY()
            command_msg.orientation.yaw   = std::atan2( R_interp(1,0), R_interp(0,0));
            command_msg.orientation.pitch = std::atan2(-R_interp(2,0), std::sqrt(R_interp(2,1)*R_interp(2,1)
                                                                                + R_interp(2,2)*R_interp(2,2)));
            command_msg.orientation.roll  = std::atan2( R_interp(2,1), R_interp(2,2));

            pub_command->publish(command_msg);

            if (finished) {
                executing = false;
                RCLCPP_INFO(get_logger(), "[ultrasound_traj_node] Step completed. "
                         "Holding target pose. Waiting for next /ee_action ...");
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
        RCLCPP_ERROR(rclcpp::get_logger("ee_incremental_motion"), "%s", error.what());
        result = 1;
    }
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return result;
}
