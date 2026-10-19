/***********************************************************************
 * Author: NearLab_MRS_KUKA
 * Filename: pose_init.cpp
 * Function: Move the KUKA LWR arm to a safe home joint configuration
 *           by publishing a single-point trajectory to
 *           joint_trajectory_controller.
 * Node:     rosrun ultrasound_trajectory pose_init
 ************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <map>

class PoseInit
{
private:
    // ROS 1: ros::init(argc, argv, "pose_init_node") + default NodeHandle
    rclcpp::Node::SharedPtr nh_ = std::make_shared<rclcpp::Node>("pose_init_node");
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_traj_;
    // ROS 2: handshake with joint_trajectory_controller through its ~/state topic
    // (see the constructor and sendTrajectory())
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr sub_jtc_state_;
    bool jtc_state_received_ = false;
    control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr jtc_state_;

    // TODO(ros2-migration): ROS 1 delivered the message for sure once the TCP connection was
    // up. In DDS the controller may match this publisher only after the first message, which
    // is then silently dropped (measured in Gazebo: first message lost 3/15, a second one 3 s
    // later from the same publisher always taken), and Humble has no API to see that match.
    // The handshake is therefore completed at application level: the controller's ~/state
    // shows that it took the trajectory (desired velocity non-zero while it interpolates, or
    // desired position already at the target). Until then the same trajectory is resent, as
    // a TCP retransmission would do.
    bool trajectoryTaken(const trajectory_msgs::msg::JointTrajectory &traj) const
    {
        if (!jtc_state_) return false;
        std::map<std::string, size_t> index;
        for (size_t i = 0; i < jtc_state_->joint_names.size(); ++i) index[jtc_state_->joint_names[i]] = i;
        bool moving = false, at_target = true;
        for (size_t j = 0; j < traj.joint_names.size(); ++j)
        {
            const auto it = index.find(traj.joint_names[j]);
            if (it == index.end() || it->second >= jtc_state_->desired.positions.size()) return false;
            if (it->second < jtc_state_->desired.velocities.size() &&
                std::fabs(jtc_state_->desired.velocities[it->second]) > 1e-6) moving = true;
            if (std::fabs(jtc_state_->desired.positions[it->second] - traj.points.back().positions[j]) > 1e-3)
                at_target = false;
        }
        return moving || at_target;
    }

    void sendTrajectory(const trajectory_msgs::msg::JointTrajectory &traj)
    {
        rclcpp::Clock steady(RCL_STEADY_TIME);
        const auto retransmission_timeout = rclcpp::Duration::from_seconds(0.5);
        jtc_state_.reset();
        pub_joint_traj_->publish(traj);
        rclcpp::Time sent = steady.now();
        while (rclcpp::ok() && !trajectoryTaken(traj))
        {
            rclcpp::spin_some(nh_);
            if (steady.now() - sent > retransmission_timeout)
            {
                RCLCPP_WARN(nh_->get_logger(), "[pose_init] joint_trajectory_controller has not taken the trajectory yet, sending it again");
                pub_joint_traj_->publish(traj);
                sent = steady.now();
            }
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }
        sub_jtc_state_.reset();
    }

public:
    // ----------------------------------------------------------------
    // initJoints()
    // Publish a single-point JointTrajectory to bring the arm to a safe
    // home configuration.
    //
    // Target joint angles (all well within joint limits):
    //   A1 =   0°  (±170°)   base centred
    //   A2 = +30°  (±120°)   shoulder slightly tilted
    //   E1 =   0°  (±170°)   elbow centred
    //   A3 = -90°  (±120°)   forearm bent downward
    //   A4 =   0°  (±170°)   wrist 1 centred
    //   A5 = +30°  (±120°)   wrist 2 slightly bent
    //   A6 =   0°  (±170°)   tool roll centred
    // ----------------------------------------------------------------
    void initJoints()
    {
        trajectory_msgs::msg::JointTrajectory traj;
        // ros::Time(0) tells the controller to start executing from the moment
        // the message is received, not from when it was stamped here.
        // This avoids the trajectory being rejected as "in the past" on real
        // hardware where FRI scheduling or network latency can introduce a gap
        // between stamp time and receipt time.
        traj.header.stamp = rclcpp::Time(0, 0, nh_->get_clock()->get_clock_type());
        traj.joint_names = {
            "lwr_a1_joint",
            "lwr_a2_joint",
            "lwr_e1_joint",
            "lwr_a3_joint",
            "lwr_a4_joint",
            "lwr_a5_joint",
            "lwr_a6_joint"
        };

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        // pt.positions = {
        //     -M_PI / 3.0,    // A1 =   0°
        //      M_PI / 6.0,    // A2 = +30°
        //      0.0,           // E1 =   0°
        //     -M_PI / 2.0,    // A3 = -90°
        //      0.0,           // A4 =   0°
        //      M_PI / 6.0,    // A5 = +30°
        //      0.0            // A6 =   0°
        // };
        pt.positions = {
            -0.43599,
             0.72318,
            -0.53115, 
            -1.39626, 
             0.40051,     
             1.11764,    
            -0.64213     
        };
        pt.velocities.assign(7, 0.0);
        pt.time_from_start = rclcpp::Duration::from_seconds(5.0);   // reach target within 5 s

        traj.points.push_back(pt);
        sendTrajectory(traj);

        RCLCPP_INFO(nh_->get_logger(), "[pose_init] Home configuration sent to joint_trajectory_controller.");
        RCLCPP_INFO(nh_->get_logger(), "[pose_init]   A1=0  A2=+30deg  E1=0  A3=-90deg  A4=0  A5=+30deg  A6=0");
        RCLCPP_INFO(nh_->get_logger(), "[pose_init] Waiting 6 s for the arm to reach home ...");
        // ROS 1 ros::Duration(6.0).sleep() follows ROS time; roscpp updates it from /clock in
        // its own thread. The rclcpp clock only advances while the node processes /clock, so
        // wait on the node clock while spinning (the node also stays alive meanwhile).
        // As ros::Duration::sleep(), a zero start time (no /clock received yet) restarts the
        // wait once the time is valid, so the full duration is always slept.
        rclcpp::Time start = nh_->now();
        while (rclcpp::ok() && (start.nanoseconds() == 0 ||
                                nh_->now() < start + rclcpp::Duration::from_seconds(6.0)))
        {
            rclcpp::spin_some(nh_);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
            if (start.nanoseconds() == 0) start = nh_->now();
        }
        RCLCPP_INFO(nh_->get_logger(), "[pose_init] Done.");
    }

    PoseInit()
    {
        // TODO(ros2-migration): the Humble joint_trajectory_controller subscribes to
        // ~/joint_trajectory, not ~/command (see MIGRATION_NOTES)
        pub_joint_traj_ = nh_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/lwr/joint_trajectory_controller/joint_trajectory", 1);

        // Wait until joint_trajectory_controller has actually subscribed.
        // A blind sleep is unreliable on real hardware: FRI initialisation and
        // controller-manager startup take longer than in simulation, so a fixed
        // 1-second wait often fires before any subscriber exists and the message
        // is silently dropped (non-latched publisher).
        // ROS 2: besides seeing the controller's subscription, wait for the controller's
        // ~/state (same topic name as in ROS 1): the controller has discovered this node too.
        sub_jtc_state_ = nh_->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/lwr/joint_trajectory_controller/state", 1,
            [this](control_msgs::msg::JointTrajectoryControllerState::ConstSharedPtr msg) {
                jtc_state_received_ = true;
                jtc_state_ = msg;
            });

        RCLCPP_INFO(nh_->get_logger(), "[pose_init] Waiting for joint_trajectory_controller to subscribe ...");
        rclcpp::Rate wait_rate(10);   // 10 Hz poll — low CPU cost
        while (rclcpp::ok() && (pub_joint_traj_->get_subscription_count() == 0 || !jtc_state_received_))
        {
            rclcpp::spin_some(nh_);
            wait_rate.sleep();
        }
        initJoints();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    PoseInit pose_init;
    rclcpp::shutdown();
    return 0;
}
