/*
 * trajectory_interpolation_node.cpp
 *
 * Interpolation between the target poses of a low-rate publisher (the ACT policy, controllers.deploy.act_policy in
 * nearlab/controllers, one target per ultrasound frame at ~10 Hz), streamed to the Cartesian impedance controller as
 * lwr_controllers/CartesianImpedancePoint at ~command_rate (100 Hz). A process of its own (and C++) so that the
 * 100 Hz loop and the inference do not share a Python interpreter (GIL).
 *
 * Each target starts a segment from the last command to the target, followed over the time between the last two
 * targets (their header stamps: the capture times of the frames, 1 / FPS), at most at ~max_linear_vel /
 * ~max_angular_vel (linear for the position, slerp for the rotation). The first target starts from the measured pose
 * and is approached at the velocity limits. Before it the node publishes nothing.
 *
 * Command interface as in scripts/drag_demonstration.py: x_fri is the lwr_7_link pose of
 * /cartesian_position/pose/end_effector itself (roll/pitch/yaw, R = Rz(yaw) Ry(pitch) Rx(roll)), in the base frame,
 * with no tool offset applied: commanding the measured pose holds the robot where it is.
 *
 * Along the z axis of the end-effector frame the probe is force controlled, as in linear_traj_node_Luca: the
 * commanded position on that axis is the measured one (no spring), f_fri.force.z = ~contact_force [N] (ramped up
 * over ~force_ramp_time from the first target) and the stiffness on that axis is ~stiffness_z. k_fri, d_fri and f_fri
 * are expressed in the end-effector frame (CartesianImpedancePoint).
 *
 * Run = process: start the node to run, Ctrl+C to stop. The node holds (measured pose, no contact force, z position
 * controlled again) on Ctrl+C, and holds then exits (code 1) on a stop condition: no target for ~target_timeout (the
 * publisher stopped, crashed or hangs, or it gets no image), a pose older than ~pose_timeout, or a measured travel
 * of more than ~max_z_travel along the force controlled axis since the first target (contact lost: nothing else
 * stops the probe under the contact force). That travel is the sum of the measured displacements along the z axis
 * of the end effector at the time of each: a scan across a curved surface with the probe kept normal to it adds
 * none. The workspace check on the targets themselves is made by their publisher. With ~dry_run the commands are
 * only published on ~/command_preview, not to the controller.
 *
 * Topic in  : ~pose_topic    (default /cartesian_position/pose/end_effector, lwr_controllers/PoseRPYStamped)
 * Topic in  : ~target_topic  (default /act_policy/target, geometry_msgs/PoseStamped, lwr_7_link in the base frame)
 * Topic out : ~command_topic (default /lwr/cartesian_impedance_controller/command, CartesianImpedancePoint)
 * Topic out : ~/command_preview (geometry_msgs/PoseStamped)
 *
 * Single-threaded: the timer and the callbacks never run at the same time, so no command is published after the hold.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lwr_controllers/msg/cartesian_impedance_point.hpp"
#include "lwr_controllers/msg/pose_rpy_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

const char *BASE_FRAME = "lwr_base_link";

struct Pose {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

Pose pose_from_rpy(const lwr_controllers::msg::PoseRPY &msg) {
    Pose pose;
    pose.p = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
    pose.q = Eigen::Quaterniond(Eigen::AngleAxisd(msg.orientation.yaw, Eigen::Vector3d::UnitZ())
                              * Eigen::AngleAxisd(msg.orientation.pitch, Eigen::Vector3d::UnitY())
                              * Eigen::AngleAxisd(msg.orientation.roll, Eigen::Vector3d::UnitX())).normalized();
    return pose;
}

Pose pose_from_msg(const geometry_msgs::msg::Pose &msg) {
    Pose pose;
    pose.p = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
    pose.q = Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    pose.q.normalize();
    return pose;
}

geometry_msgs::msg::Pose pose_to_msg(const Pose &pose) {
    geometry_msgs::msg::Pose msg;
    msg.position.x = pose.p.x();
    msg.position.y = pose.p.y();
    msg.position.z = pose.p.z();
    msg.orientation.x = pose.q.x();
    msg.orientation.y = pose.q.y();
    msg.orientation.z = pose.q.z();
    msg.orientation.w = pose.q.w();
    return msg;
}

// Between a (alpha = 0) and b (alpha = 1): linear for the position, slerp (shortest arc) for the rotation
Pose interpolate(const Pose &a, const Pose &b, double alpha) {
    Pose pose;
    pose.p = a.p + alpha * (b.p - a.p);
    pose.q = a.q.slerp(alpha, b.q).normalized();
    return pose;
}

// From current towards target by at most max_linear [m] and max_angle [rad] (utils.rigid_transform.step_towards)
Pose step_towards(const Pose &current, const Pose &target, double max_linear, double max_angle) {
    const double distance = (target.p - current.p).norm();
    const double angle = current.q.angularDistance(target.q);
    double alpha = 1.0;
    if (distance > max_linear) alpha = std::min(alpha, max_linear / distance);
    if (angle > max_angle) alpha = std::min(alpha, max_angle / angle);
    return alpha >= 1.0 ? target : interpolate(current, target, alpha);
}

double seconds(const rclcpp::Time &earlier, const rclcpp::Time &later) { return (later - earlier).seconds(); }

}  // namespace

class TrajectoryInterpolationNode : public rclcpp::Node {
public:
    TrajectoryInterpolationNode() : rclcpp::Node("trajectory_interpolation") {
        const auto pose_topic = declare_parameter<std::string>("pose_topic", "/cartesian_position/pose/end_effector");
        const auto target_topic = declare_parameter<std::string>("target_topic", "/act_policy/target");
        const auto command_topic =
            declare_parameter<std::string>("command_topic", "/lwr/cartesian_impedance_controller/command");

        // Impedance of the commands, defaults of ee_incremental_impedance_node
        stiffness_linear_ = declare_parameter("stiffness_linear", 1500.0);    // [N/m]
        stiffness_angular_ = declare_parameter("stiffness_angular", 250.0);   // [Nm/rad]
        damping_ = declare_parameter("damping", 0.8);                         // normalized damping ratio
        // Force control along the z axis of the end-effector frame, positive along +z as in linear_traj_node_Luca
        contact_force_ = declare_parameter("contact_force", 0.0);             // [N]
        force_ramp_time_ = declare_parameter("force_ramp_time", 2.0);         // [s] from 0 to contact_force
        stiffness_z_ = declare_parameter("stiffness_z", 200.0);               // [N/m] on the force controlled axis
        const double max_contact_force = declare_parameter("max_contact_force", 15.0);  // [N] refused above
        if (std::abs(contact_force_) > max_contact_force)
            throw std::invalid_argument("|contact_force| > max_contact_force");
        if (force_ramp_time_ < 0.0 || stiffness_z_ < 0.0)
            throw std::invalid_argument("force_ramp_time and stiffness_z must be >= 0");
        // [m] measured travel along z of the end effector since the first target before holding (contact lost)
        max_z_travel_ = declare_parameter("max_z_travel", 0.03);
        if (max_z_travel_ <= 0.0) throw std::invalid_argument("max_z_travel must be > 0");

        command_rate_ = declare_parameter("command_rate", 100.0);             // [Hz]
        max_linear_vel_ = declare_parameter("max_linear_vel", 0.02);          // [m/s]
        max_angular_vel_ = declare_parameter("max_angular_vel", 0.2);         // [rad/s]
        pose_timeout_ = declare_parameter("pose_timeout", 0.5);               // [s]
        // [s] without a target (3 frames at 10 FPS) before holding for good; also the longest segment followed
        target_timeout_ = declare_parameter("target_timeout", 0.3);
        dry_run_ = declare_parameter("dry_run", false);
        if (command_rate_ <= 0.0 || target_timeout_ <= 0.0)
            throw std::invalid_argument("command_rate and target_timeout must be > 0");

        pose_sub_ = create_subscription<lwr_controllers::msg::PoseRPYStamped>(
            pose_topic, 10, [this](lwr_controllers::msg::PoseRPYStamped::ConstSharedPtr msg) { on_pose(*msg); });
        target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            target_topic, 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { on_target(*msg); });
        command_pub_ = create_publisher<lwr_controllers::msg::CartesianImpedancePoint>(command_topic, 10);
        preview_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/command_preview", 10);

        // On the node clock, as the stamps (simulation time with use_sim_time)
        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / command_rate_),
                                      [this]() { command_step(); });
        if (contact_force_ == 0.0)
            RCLCPP_WARN(get_logger(), "contact_force is 0: the z axis of the end effector is free, nothing presses "
                                      "the probe");
        RCLCPP_INFO(get_logger(), "Waiting for the first target on %s (%s, contact force %.1f N)",
                    target_topic.c_str(),
                    dry_run_ ? "DRY RUN, nothing is sent to the controller" : ("commands on " + command_topic).c_str(),
                    contact_force_);
    }

    // Ctrl+C: hold where the robot is before the node goes away
    void shut_down() { hold("node shut down"); }

    // A stop condition ended the run: the node exits
    bool holding() const { return state_ == State::holding; }

private:
    enum class State { waiting, running, holding };

    struct Segment {
        rclcpp::Time start;     // first target: start of the force ramp
        Pose command;           // last commanded pose
        Pose segment_start;     // command when the target came
        Pose target;
        rclcpp::Time segment_time;
        double segment_duration = 0.0;  // [s], 0: at the velocity limits
        rclcpp::Time last_target_arrival;
        rclcpp::Time last_target_stamp;  // header stamp (capture time of the frame)
        Pose last_pose;         // measured pose of the last command step
        double z_travel = 0.0;  // [m] measured travel along z of the end effector since the first target
    };

    void on_pose(const lwr_controllers::msg::PoseRPYStamped &msg) {
        pose_ = pose_from_rpy(msg.pose);
        pose_arrival_ = now();
    }

    bool pose_fresh(const rclcpp::Time &t) const {
        return pose_arrival_.has_value() && seconds(*pose_arrival_, t) < pose_timeout_;
    }

    void on_target(const geometry_msgs::msg::PoseStamped &msg) {
        if (state_ == State::holding) return;
        const auto t = now();
        const rclcpp::Time stamp(msg.header.stamp, RCL_ROS_TIME);
        const Pose target = pose_from_msg(msg.pose);
        if (state_ == State::waiting) {
            if (!pose_fresh(t)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Target ignored: no recent end-effector pose");
                return;
            }
            // From the measured pose, at the velocity limits
            Segment s{t, *pose_, *pose_, target, t, 0.0, t, stamp, *pose_, 0.0};
            run_ = s;
            state_ = State::running;
            RCLCPP_INFO(get_logger(), "First target: running from the current pose");
            return;
        }
        Segment &s = *run_;
        // Over the frame period of the publisher (time between the frames of the last two targets)
        s.segment_duration = std::clamp(seconds(s.last_target_stamp, stamp), 0.0, target_timeout_);
        s.segment_start = s.command;
        s.segment_time = t;
        s.target = target;
        s.last_target_arrival = t;
        s.last_target_stamp = stamp;
    }

    void command_step() {
        if (state_ != State::running) return;
        Segment &s = *run_;
        const auto t = now();
        if (!pose_fresh(t)) {
            hold("no recent end-effector pose");
            return;
        }
        if (seconds(s.last_target_arrival, t) > target_timeout_) {
            hold("no target for " + std::to_string(seconds(s.last_target_arrival, t)) + " s (> target_timeout)");
            return;
        }
        // z of the end effector is force controlled: a measured travel along it beyond max_z_travel means the
        // contact was lost
        const Pose &measured = *pose_;
        s.z_travel += (measured.q * Eigen::Vector3d::UnitZ()).dot(measured.p - s.last_pose.p);
        s.last_pose = measured;
        if (std::abs(s.z_travel) > max_z_travel_) {
            hold("measured pose moved " + std::to_string(s.z_travel * 1000.0)
                 + " mm along z of the end effector (> max_z_travel)");
            return;
        }
        // Along the segment at the speed that reaches the target in one frame period (no move-and-wait staircase at
        // the frame rate), within the velocity limits
        const double alpha = s.segment_duration > 0.0
                                 ? std::clamp(seconds(s.segment_time, t) / s.segment_duration, 0.0, 1.0)
                                 : 1.0;
        const Pose desired = interpolate(s.segment_start, s.target, alpha);
        const double dt = 1.0 / command_rate_;
        s.command = step_towards(s.command, desired, max_linear_vel_ * dt, max_angular_vel_ * dt);
        const double elapsed = seconds(s.start, t);
        const double ramp = force_ramp_time_ <= 0.0 ? 1.0 : std::min(1.0, elapsed / force_ramp_time_);
        publish_command(s.command, ramp * contact_force_);
    }

    // For good: the run is over, the node exits. Nothing to hold before the first target (nothing was commanded)
    void hold(const std::string &reason) {
        if (state_ != State::running) return;
        state_ = State::holding;
        // Where the robot is, so the impedance does not pull towards the last target; the contact force is
        // released and z is position controlled again
        if (pose_.has_value()) publish_command(*pose_, std::nullopt);
        RCLCPP_WARN(get_logger(), "Holding the measured pose: %s. Start the node again to run", reason.c_str());
    }

    // pose: commanded lwr_7_link pose in the base frame. contact_force: force [N] along z of the end effector, that
    // axis force controlled; nullopt: position control on all axes
    void publish_command(Pose pose, std::optional<double> contact_force) {
        if (contact_force && pose_.has_value()) {
            // Commanded pose in the measured end-effector frame, its z there replaced by the measured one (0): no
            // spring along that axis, only the commanded force
            const Pose &measured = *pose_;
            Eigen::Vector3d in_measured = measured.q.conjugate() * (pose.p - measured.p);
            in_measured.z() = 0.0;
            pose.p = measured.p + measured.q * in_measured;
        }
        geometry_msgs::msg::PoseStamped preview;
        preview.header.stamp = now();
        preview.header.frame_id = BASE_FRAME;
        preview.pose = pose_to_msg(pose);
        preview_pub_->publish(preview);
        if (dry_run_) return;

        lwr_controllers::msg::CartesianImpedancePoint msg;
        msg.header = preview.header;
        msg.x_fri = preview.pose;
        msg.k_fri.x = msg.k_fri.y = msg.k_fri.z = stiffness_linear_;
        msg.k_fri.rx = msg.k_fri.ry = msg.k_fri.rz = stiffness_angular_;
        msg.d_fri.x = msg.d_fri.y = msg.d_fri.z = damping_;
        msg.d_fri.rx = msg.d_fri.ry = msg.d_fri.rz = damping_;
        // Force control along z of the end effector; otherwise f_fri stays zero, no additional force
        if (contact_force) {
            msg.k_fri.z = stiffness_z_;
            msg.f_fri.force.z = *contact_force;
        }
        command_pub_->publish(msg);
    }

    double stiffness_linear_, stiffness_angular_, damping_;
    double contact_force_, force_ramp_time_, stiffness_z_, max_z_travel_;
    double command_rate_, max_linear_vel_, max_angular_vel_, pose_timeout_, target_timeout_;
    bool dry_run_;

    State state_ = State::waiting;
    std::optional<Pose> pose_;  // newest measured pose
    std::optional<rclcpp::Time> pose_arrival_;
    std::optional<Segment> run_;

    rclcpp::Subscription<lwr_controllers::msg::PoseRPYStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<lwr_controllers::msg::CartesianImpedancePoint>::SharedPtr command_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr preview_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    // No rclcpp signal handler: on Ctrl+C the context stays valid so the hold command can still be published
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<TrajectoryInterpolationNode>();
    static std::atomic<bool> interrupted{false};
    std::signal(SIGINT, [](int) { interrupted = true; });
    std::signal(SIGTERM, [](int) { interrupted = true; });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !interrupted && !node->holding()) executor.spin_once(std::chrono::milliseconds(50));
    // Non-zero on a stop condition, as act_policy: the launch file stops the policy with it
    const bool stopped = node->holding();
    node->shut_down();
    // Time for DDS to deliver the hold command before the participant goes away
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    rclcpp::shutdown();
    return stopped ? 1 : 0;
}
