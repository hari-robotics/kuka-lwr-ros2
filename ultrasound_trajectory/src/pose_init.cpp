#include "ultrasound_trajectory/ros2_support.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class PoseInit : public rclcpp::Node {
public:
    using Action = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
    using SteadyClock = std::chrono::steady_clock;

    PoseInit() : Node("pose_init") {
        const auto action_name = ultrasound_trajectory::startup_parameter(*this,
            "action_name", "/lwr/joint_trajectory_controller/follow_joint_trajectory");
        duration_ = ultrasound_trajectory::startup_parameter(*this, "duration", 5.0);
        server_timeout_ = ultrasound_trajectory::startup_parameter(*this, "server_timeout", 15.0);
        execution_timeout_ = ultrasound_trajectory::startup_parameter(*this,
            "execution_timeout", duration_ + 30.0);
        position_tolerance_ = ultrasound_trajectory::startup_parameter(*this,
            "position_tolerance", 0.01);
        target_ = declare_parameter<std::vector<double>>("joint_positions",
            std::vector<double>{-0.43599, 0.72318, -0.53115, -1.39626, 0.40051, 1.11764, -0.64213});
        if (target_.size() != 7 || !positive(duration_) || !positive(server_timeout_) ||
            !positive(execution_timeout_) || execution_timeout_ <= duration_ ||
            !positive(position_tolerance_)) {
            throw std::invalid_argument(
                "Expected seven joint_positions, positive timeouts, and execution_timeout > duration");
        }
        for (double position : target_) {
            if (!std::isfinite(position)) {throw std::invalid_argument("Non-finite joint target");}
        }
        client_ = rclcpp_action::create_client<Action>(this, action_name);
        wall_start_ = SteadyClock::now();
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PoseInit::tick, this));
        RCLCPP_INFO(get_logger(), "Waiting for trajectory action server on %s", action_name.c_str());
    }
    int exit_code() const {return exit_code_;}

private:
    static bool positive(double value) {return std::isfinite(value) && value > 0.0;}
    double wall_elapsed() const {
        return std::chrono::duration<double>(SteadyClock::now() - wall_start_).count();
    }
    void finish(int code) {
        exit_code_ = code;
        timer_->cancel();
        rclcpp::shutdown();
    }
    void tick() {
        if (cancel_requested_) {
            if (wall_elapsed() > 5.0) {
                RCLCPP_ERROR(get_logger(), "Timed out waiting for cancellation result");
                finish(1);
            }
            return;
        }
        if (goal_handle_) {
            if ((now() - motion_start_).seconds() > execution_timeout_) {
                RCLCPP_ERROR(get_logger(), "Execution timed out; requesting goal cancellation");
                cancel_requested_ = true;
                wall_start_ = SteadyClock::now();
                client_->async_cancel_goal(goal_handle_);
            }
            return;
        }
        if (sent_) {
            if (wall_elapsed() > server_timeout_) {
                RCLCPP_ERROR(get_logger(), "No goal acceptance response from controller");
                finish(1);
            }
            return;
        }
        if (!client_->action_server_is_ready()) {
            if (wall_elapsed() > server_timeout_) {
                RCLCPP_ERROR(get_logger(), "Trajectory action server unavailable; check controller and ROS domain");
                finish(1);
            }
            return;
        }
        Action::Goal goal;
        goal.trajectory.joint_names = {"lwr_a1_joint", "lwr_a2_joint", "lwr_e1_joint", "lwr_a3_joint",
                                      "lwr_a4_joint", "lwr_a5_joint", "lwr_a6_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = target_;
        point.velocities.assign(7, 0.0);
        point.time_from_start = rclcpp::Duration::from_seconds(duration_);
        goal.trajectory.points.push_back(point);
        // The controller's default position tolerance may be disabled (zero).
        // Require actual joint positions to reach this goal before reporting success.
        for (const auto & name : goal.trajectory.joint_names) {
            control_msgs::msg::JointTolerance tolerance;
            tolerance.name = name;
            tolerance.position = position_tolerance_;
            goal.goal_tolerance.push_back(tolerance);
        }
        goal.goal_time_tolerance = rclcpp::Duration::from_seconds(execution_timeout_ - duration_);
        // Zero stamp starts on receipt, using the controller's clock.
        rclcpp_action::Client<Action>::SendGoalOptions options;
        options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
            if (!handle) {
                RCLCPP_ERROR(get_logger(), "Controller rejected initialization goal");
                finish(1);
                return;
            }
            goal_handle_ = handle;
            motion_start_ = now();
            RCLCPP_INFO(get_logger(), "Controller accepted initialization goal (duration %.2f s)", duration_);
        };
        options.result_callback = [this](const GoalHandle::WrappedResult & result) {
            if (!cancel_requested_ && result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                result.result && result.result->error_code == Action::Result::SUCCESSFUL) {
                RCLCPP_INFO(get_logger(), "Controller reports initialization succeeded");
                finish(0);
            } else {
                RCLCPP_ERROR(get_logger(), "Initialization failed: result=%d, error=%d, %s",
                    static_cast<int>(result.code), result.result ? result.result->error_code : -1,
                    result.result ? result.result->error_string.c_str() : "No result received");
                finish(1);
            }
        };
        sent_ = true;
        wall_start_ = SteadyClock::now();
        client_->async_send_goal(goal, options);
        RCLCPP_INFO(get_logger(), "Initialization goal requested; waiting for controller acceptance");
    }

    double duration_{0}, server_timeout_{0}, execution_timeout_{0}, position_tolerance_{0};
    std::vector<double> target_;
    bool sent_{false}, cancel_requested_{false};
    int exit_code_{1};
    SteadyClock::time_point wall_start_;
    rclcpp::Time motion_start_{0, 0, RCL_ROS_TIME};
    GoalHandle::SharedPtr goal_handle_;
    rclcpp_action::Client<Action>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    int result = 1;
    try {
        auto node = std::make_shared<PoseInit>();
        rclcpp::spin(node);
        result = node->exit_code();
    } catch (const std::exception & error) {
        RCLCPP_ERROR(rclcpp::get_logger("pose_init"), "%s", error.what());
    }
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return result;
}
