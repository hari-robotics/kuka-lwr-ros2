#include "ultrasound_trajectory/ros2_support.hpp"
#include <functional>
#include <rclcpp/create_timer.hpp>
/*
 * tool_relative_motion_node.cpp
 *
 * Moves the KUKA LWR 4+ end-effector by translation/rotation deltas
 * expressed in the end-effector's OWN coordinate frame, following
 * KUKA_LWR4_Tool_Relative_Motion_Development_Guide.md. Builds a KDL
 * chain from robot_description, gates every waypoint sent to the
 * legacy lwr_controllers/OneTaskInverseKinematics controller through a
 * damped weighted least-squares joint-space safety check (guide
 * Section 5), and runs the guide's IDLE/PLANNING/EXECUTING/SETTLING/
 * HOLD/FAULT state machine (guide Section 3.2).
 *
 * Topic in  : ~move          (geometry_msgs/Twist)   tool-frame delta
 *             linear.xyz = dx,dy,dz [m], angular.xyz = droll,dpitch,dyaw [rad]
 * Topic in  : ~cancel        (std_msgs/Empty)
 * Topic in  : ~fault_reset   (std_msgs/Empty)
 * Topic in  : joint_state_topic (sensor_msgs/JointState), default /joint_states
 * Topic in  : cross_check_pose_topic (lwr_controllers/PoseRPY), diagnostic only
 * Topic out : legacy_command_topic (lwr_controllers/PoseRPY), id=0 always
 *             default /lwr/one_task_inverse_kinematics/command
 * Topic out : ~status        (std_msgs/String)
 *
 * monitor_only defaults to true: commands are computed and logged but
 * never published to the legacy topic until explicitly set false
 * (guide Section 9's staged commissioning).
 *
 * Guide's publisher-ownership rule: this node and
 * ee_incremental_motion.cpp / ultrasound_traj_node.cpp must never run
 * simultaneously -- both publish to the same legacy command topic.
 */

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <lwr_controllers/msg/pose_rpy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include "ultrasound_trajectory/diagnostics.h"
#include "ultrasound_trajectory/ik_safety_gate.h"
#include "ultrasound_trajectory/joint_state_mapper.h"
#include "ultrasound_trajectory/legacy_pose_publisher.h"
#include "ultrasound_trajectory/motion_state_machine.h"
#include "ultrasound_trajectory/trm_types.h"

namespace {

// Sanity-check reference only (guide's own commissioning values are
// site-specific; this is just to WARN if the parsed URDF disagrees
// with the values confirmed for this workspace, not a source of
// truth -- joint_limits actually used come from urdf::Model below).
struct RefLimit {
    const char* name;
    double lower{0};
    double upper{0};
};
const RefLimit kReferenceLimits[] = {
    {"lwr_a1_joint", -2.96706, 2.96706}, {"lwr_a2_joint", -2.09440, 2.09440},
    {"lwr_e1_joint", -2.96706, 2.96706}, {"lwr_a3_joint", -2.09440, 2.09440},
    {"lwr_a4_joint", -2.96706, 2.96706}, {"lwr_a5_joint", -2.09440, 2.09440},
    {"lwr_a6_joint", -2.96706, 2.96706},
};

}  // namespace

class TrmNode : public rclcpp::Node {
public:
    TrmNode() : rclcpp::Node("tool_relative_motion_node") {
        if (!loadParams()) {throw std::invalid_argument("Invalid motion parameters");}
        const auto description = ultrasound_trajectory::startup_parameter(*this, "robot_description", std::string(""));
        const auto topic = ultrasound_trajectory::startup_parameter(*this, "robot_description_topic", std::string("/robot_description"));
        if (!description.empty()) {
            if (!initialize(description)) {throw std::invalid_argument("Invalid robot_description");}
        } else {
            description_sub_ = create_subscription<std_msgs::msg::String>(
                topic, rclcpp::QoS(1).reliable().transient_local(),
                [this](std_msgs::msg::String::ConstSharedPtr msg) {
                    if (!ok_ && !initialize(msg->data)) {
                        RCLCPP_ERROR(get_logger(), "Waiting for a valid robot_description");
                    }
                });
            RCLCPP_INFO(get_logger(), "Waiting for robot description on %s", topic.c_str());
        }
    }

    bool initialize(const std::string & description) {
        if (!buildKinematics(description)) {return false;}
        joint_mapper_.setClock(get_clock());
        joint_mapper_.configure(ik_gate_.jointNames());

        pub_legacy_ = create_publisher<lwr_controllers::msg::PoseRPY>(legacy_command_topic_, 1);
        pub_status_ = create_publisher<std_msgs::msg::String>("~/status", 1);

        pose_pub_.reset(new trm::LegacyPosePublisher(pub_legacy_, monitor_only_));
        diagnostics_.reset(new trm::Diagnostics(pub_status_));
        state_machine_.reset(new trm::MotionStateMachine(joint_mapper_, ik_gate_, *pose_pub_,
                                                           *diagnostics_, limits_));

        sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(joint_state_topic_, rclcpp::SensorDataQoS(), std::bind(&trm::JointStateMapper::update, &joint_mapper_, std::placeholders::_1));
        sub_move_ = create_subscription<geometry_msgs::msg::Twist>("~/move", rclcpp::QoS(1), std::bind(&TrmNode::onMove, this, std::placeholders::_1));
        sub_cancel_ = create_subscription<std_msgs::msg::Empty>("~/cancel", rclcpp::QoS(1), std::bind(&TrmNode::onCancel, this, std::placeholders::_1));
        sub_reset_ = create_subscription<std_msgs::msg::Empty>("~/fault_reset", rclcpp::QoS(1), std::bind(&TrmNode::onFaultReset, this, std::placeholders::_1));
        if (enable_pose_crosscheck_) {
            sub_cross_check_ = create_subscription<lwr_controllers::msg::PoseRPY>(cross_check_pose_topic_, rclcpp::SensorDataQoS(), std::bind(&TrmNode::onCrossCheckPose, this, std::placeholders::_1));
        }

        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / limits_.publish_rate), std::bind(&TrmNode::onTick, this));
        diag_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / limits_.diagnostics_rate), std::bind(&TrmNode::onDiagTimer, this));

        RCLCPP_INFO_STREAM(get_logger(), "[tool_relative_motion_node] Ready. root_link=" << root_link_
                         << " tip_link=" << tip_link_ << " monitor_only=" << (monitor_only_ ? "true" : "false")
                         << " joints=" << ik_gate_.numJoints());
        ok_ = true;
        return true;
    }

    bool ok() const { return ok_; }

private:
    bool loadParams() {
        root_link_ = ultrasound_trajectory::startup_parameter(*this, "root_link", std::string("lwr_base_link"));
        tip_link_ = ultrasound_trajectory::startup_parameter(*this, "tip_link", std::string("lwr_7_link"));
        joint_state_topic_ = ultrasound_trajectory::startup_parameter(*this, "joint_state_topic", std::string("/lwr/joint_states"));
        legacy_command_topic_ = ultrasound_trajectory::startup_parameter(*this, "legacy_command_topic", std::string("/lwr/one_task_inverse_kinematics/command"));
        cross_check_pose_topic_ = ultrasound_trajectory::startup_parameter(*this, "cross_check_pose_topic", std::string("/cartesian_position/pose/end_effector"));
        enable_pose_crosscheck_ = ultrasound_trajectory::startup_parameter(*this, "enable_pose_crosscheck", true);
        monitor_only_ = ultrasound_trajectory::startup_parameter(*this, "monitor_only", true);

        limits_.publish_rate = ultrasound_trajectory::startup_parameter(*this, "publish_rate", limits_.publish_rate);
        limits_.diagnostics_rate = ultrasound_trajectory::startup_parameter(*this, "diagnostics_rate", limits_.diagnostics_rate);
        limits_.t_min = ultrasound_trajectory::startup_parameter(*this, "t_min", limits_.t_min);
        limits_.max_duration_stretch_factor = ultrasound_trajectory::startup_parameter(*this, "max_duration_stretch_factor", limits_.max_duration_stretch_factor);

        limits_.request_translation_max = ultrasound_trajectory::startup_parameter(*this, "request_translation_max", limits_.request_translation_max);
        limits_.request_rotation_max = ultrasound_trajectory::startup_parameter(*this, "request_rotation_max", limits_.request_rotation_max);
        limits_.v_trans_max = ultrasound_trajectory::startup_parameter(*this, "v_trans_max", limits_.v_trans_max);
        limits_.a_trans_max = ultrasound_trajectory::startup_parameter(*this, "a_trans_max", limits_.a_trans_max);
        limits_.v_rot_max = ultrasound_trajectory::startup_parameter(*this, "v_rot_max", limits_.v_rot_max);
        limits_.a_rot_max = ultrasound_trajectory::startup_parameter(*this, "a_rot_max", limits_.a_rot_max);

        limits_.dq_waypoint_max = ultrasound_trajectory::startup_parameter(*this, "dq_waypoint_max", limits_.dq_waypoint_max);
        limits_.qdot_pred_max = ultrasound_trajectory::startup_parameter(*this, "qdot_pred_max", limits_.qdot_pred_max);
        limits_.qddot_pred_max = ultrasound_trajectory::startup_parameter(*this, "qddot_pred_max", limits_.qddot_pred_max);
        limits_.joint_limit_margin = ultrasound_trajectory::startup_parameter(*this, "joint_limit_margin", limits_.joint_limit_margin);

        limits_.feedback_timeout = ultrasound_trajectory::startup_parameter(*this, "feedback_timeout", limits_.feedback_timeout);

        limits_.following_position_gate = ultrasound_trajectory::startup_parameter(*this, "following_position_gate", limits_.following_position_gate);
        limits_.following_orientation_gate = ultrasound_trajectory::startup_parameter(*this, "following_orientation_gate", limits_.following_orientation_gate);
        limits_.following_error_abort_position = ultrasound_trajectory::startup_parameter(*this, "following_error_abort_position", limits_.following_error_abort_position);
        limits_.following_error_abort_orientation = ultrasound_trajectory::startup_parameter(*this, "following_error_abort_orientation", limits_.following_error_abort_orientation);
        limits_.following_error_abort_duration = ultrasound_trajectory::startup_parameter(*this, "following_error_abort_duration", limits_.following_error_abort_duration);

        limits_.settle_position_threshold = ultrasound_trajectory::startup_parameter(*this, "settle_position_threshold", limits_.settle_position_threshold);
        limits_.settle_orientation_threshold = ultrasound_trajectory::startup_parameter(*this, "settle_orientation_threshold", limits_.settle_orientation_threshold);
        limits_.settle_joint_speed_threshold = ultrasound_trajectory::startup_parameter(*this, "settle_joint_speed_threshold", limits_.settle_joint_speed_threshold);
        limits_.settle_hold_duration = ultrasound_trajectory::startup_parameter(*this, "settle_hold_duration", limits_.settle_hold_duration);
        limits_.settle_timeout = ultrasound_trajectory::startup_parameter(*this, "settle_timeout", limits_.settle_timeout);

        limits_.ds_min = ultrasound_trajectory::startup_parameter(*this, "ds_min", limits_.ds_min);
        limits_.sigma_threshold = ultrasound_trajectory::startup_parameter(*this, "sigma_threshold", limits_.sigma_threshold);
        limits_.sigma_min_hard_stop = ultrasound_trajectory::startup_parameter(*this, "sigma_min_hard_stop", limits_.sigma_min_hard_stop);
        limits_.ik_lambda_max = ultrasound_trajectory::startup_parameter(*this, "ik_lambda_max", limits_.ik_lambda_max);
        limits_.joint_limit_weight_gain = ultrasound_trajectory::startup_parameter(*this, "joint_limit_weight_gain", limits_.joint_limit_weight_gain);
        limits_.elbow_weight_gain = ultrasound_trajectory::startup_parameter(*this, "elbow_weight_gain", limits_.elbow_weight_gain);
        limits_.null_space_gain = ultrasound_trajectory::startup_parameter(*this, "null_space_gain", limits_.null_space_gain);
        limits_.continuity_jump_multiple = ultrasound_trajectory::startup_parameter(*this, "continuity_jump_multiple", limits_.continuity_jump_multiple);

        const double numeric_limits[] = {limits_.publish_rate, limits_.diagnostics_rate, limits_.t_min, limits_.max_duration_stretch_factor, limits_.request_translation_max, limits_.request_rotation_max, limits_.v_trans_max, limits_.a_trans_max, limits_.v_rot_max, limits_.a_rot_max, limits_.dq_waypoint_max, limits_.qdot_pred_max, limits_.qddot_pred_max, limits_.joint_limit_margin, limits_.feedback_timeout, limits_.following_position_gate, limits_.following_orientation_gate, limits_.following_error_abort_position, limits_.following_error_abort_orientation, limits_.following_error_abort_duration, limits_.settle_position_threshold, limits_.settle_orientation_threshold, limits_.settle_joint_speed_threshold, limits_.settle_hold_duration, limits_.settle_timeout, limits_.ds_min, limits_.sigma_threshold, limits_.sigma_min_hard_stop, limits_.ik_lambda_max, limits_.joint_limit_weight_gain, limits_.elbow_weight_gain, limits_.null_space_gain, limits_.continuity_jump_multiple};
        for (double value : numeric_limits) {
            if (!std::isfinite(value) || value < 0.0) {return false;}
        }
        if (limits_.publish_rate <= 0.0 || limits_.diagnostics_rate <= 0.0 ||
            limits_.feedback_timeout <= 0.0 || limits_.t_min <= 0.0 ||
            limits_.v_trans_max <= 0.0 || limits_.a_trans_max <= 0.0 ||
            limits_.v_rot_max <= 0.0 || limits_.a_rot_max <= 0.0 ||
            limits_.max_duration_stretch_factor < 1.0 || limits_.ds_min <= 0.0 ||
            limits_.ds_min > 1.0) {return false;}
        return true;
    }

    bool buildKinematics(const std::string & description) {
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(description, tree)) {
            RCLCPP_FATAL(get_logger(), "[tool_relative_motion_node] Failed to parse robot_description into a KDL::Tree.");
            return false;
        }

        KDL::Chain chain;
        if (!tree.getChain(root_link_, tip_link_, chain)) {
            RCLCPP_FATAL_STREAM(get_logger(), "[tool_relative_motion_node] Could not extract KDL chain from "
                              << root_link_ << " to " << tip_link_);
            return false;
        }

        std::vector<std::string> joint_names;
        for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
            const KDL::Joint& joint = chain.getSegment(i).getJoint();
            if (joint.getType() != KDL::Joint::None) {
                joint_names.push_back(joint.getName());
            }
        }
        if (joint_names.empty()) {
            RCLCPP_FATAL(get_logger(), "[tool_relative_motion_node] KDL chain has no active joints.");
            return false;
        }

        urdf::Model model;
        if (!model.initString(description)) {
            RCLCPP_FATAL(get_logger(), "[tool_relative_motion_node] Failed to parse robot_description into a urdf::Model.");
            return false;
        }

        std::vector<trm::IkSafetyGate::JointLimit> joint_limits;
        joint_limits.reserve(joint_names.size());
        for (const auto& name : joint_names) {
            urdf::JointConstSharedPtr joint = model.getJoint(name);
            if (!joint || !joint->limits) {
                RCLCPP_FATAL_STREAM(get_logger(), 
                    "[tool_relative_motion_node] URDF joint '" << name << "' has no position/velocity limits.");
                return false;
            }
            trm::IkSafetyGate::JointLimit lim;
            lim.lower = joint->limits->lower;
            lim.upper = joint->limits->upper;
            lim.velocity = joint->limits->velocity;
            joint_limits.push_back(lim);

            for (const auto& ref : kReferenceLimits) {
                if (name == ref.name) {
                    if (std::fabs(lim.lower - ref.lower) > 1e-3 || std::fabs(lim.upper - ref.upper) > 1e-3) {
                        RCLCPP_WARN_STREAM(get_logger(), "[tool_relative_motion_node] URDF limits for "
                                        << name << " (" << lim.lower << ".." << lim.upper
                                        << ") differ from the reference values recorded for this "
                                           "workspace ("
                                        << ref.lower << ".." << ref.upper << ") -- confirm URDF/site match.");
                    }
                    break;
                }
            }
        }

        std::string error;
        if (!ik_gate_.configure(chain, joint_names, joint_limits, limits_, error)) {
            RCLCPP_FATAL_STREAM(get_logger(), "[tool_relative_motion_node] IkSafetyGate::configure failed: " << error);
            return false;
        }

        return true;
    }

    void onMove(const geometry_msgs::msg::Twist::ConstSharedPtr& msg) {
        trm::MoveRequest req;
        req.dx = msg->linear.x;
        req.dy = msg->linear.y;
        req.dz = msg->linear.z;
        req.droll = msg->angular.x;
        req.dpitch = msg->angular.y;
        req.dyaw = msg->angular.z;
        req.requested_duration = 0.0;  // planner chooses (guide 4.3)

        if (!state_machine_->submit(req)) {
            RCLCPP_WARN_STREAM(get_logger(), "[tool_relative_motion_node] ~move rejected: not IDLE (state="
                             << trm::toString(state_machine_->state()) << ")");
        }
    }

    void onCancel(const std_msgs::msg::Empty::ConstSharedPtr&) { state_machine_->cancel(); }
    void onFaultReset(const std_msgs::msg::Empty::ConstSharedPtr&) { state_machine_->reset(); }

    void onCrossCheckPose(const lwr_controllers::msg::PoseRPY::ConstSharedPtr& msg) {
        last_cross_check_ = *msg;
        have_cross_check_ = true;
    }

    void onTick() {
        state_machine_->tick(now());
        crossCheckIfDue();
    }

    void onDiagTimer() { diagnostics_->publish(); }

    // Diagnostic-only comparison of our own FK against the separately
    // published /cartesian_position/pose/end_effector -- catches a
    // root/tip/URDF mismatch early. Never gates the safety path (guide
    // 3.1: the start pose comes from fk(q_measured), not this topic).
    void crossCheckIfDue() {
        if (!enable_pose_crosscheck_ || !have_cross_check_) return;

        KDL::JntArray q;
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        if (!joint_mapper_.snapshot(q, stamp)) return;

        const KDL::Frame T_ours = ik_gate_.fk(q);
        const KDL::Vector p_theirs(last_cross_check_.position.x, last_cross_check_.position.y,
                                    last_cross_check_.position.z);
        const double pos_diff = (T_ours.p - p_theirs).Norm();

        constexpr double kCrossCheckPosTolerance = 0.01;  // 1 cm, diagnostic only
        if (pos_diff > kCrossCheckPosTolerance) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "[tool_relative_motion_node] FK cross-check mismatch vs %s: %.4f m "
                               "(check root_link/tip_link/URDF match the legacy controller's tip_name)",
                               cross_check_pose_topic_.c_str(), pos_diff);
        }
    }

    std::string root_link_, tip_link_;
    std::string joint_state_topic_, legacy_command_topic_, cross_check_pose_topic_;
    bool enable_pose_crosscheck_ = true;
    bool monitor_only_ = true;

    trm::TrmLimits limits_;
    trm::IkSafetyGate ik_gate_;
    trm::JointStateMapper joint_mapper_;

    std::unique_ptr<trm::LegacyPosePublisher> pose_pub_;
    std::unique_ptr<trm::Diagnostics> diagnostics_;
    std::unique_ptr<trm::MotionStateMachine> state_machine_;

    rclcpp::Publisher<lwr_controllers::msg::PoseRPY>::SharedPtr pub_legacy_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_move_;
rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_cancel_;
rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_reset_;
rclcpp::Subscription<lwr_controllers::msg::PoseRPY>::SharedPtr sub_cross_check_;
    rclcpp::TimerBase::SharedPtr timer_, diag_timer_;

    lwr_controllers::msg::PoseRPY last_cross_check_;
    bool have_cross_check_ = false;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
    bool ok_ = false;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<TrmNode>());
    } catch (const std::exception & error) {
        RCLCPP_ERROR(rclcpp::get_logger("tool_relative_motion_node"), "%s", error.what());
        result = 1;
    }
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return result;
}
