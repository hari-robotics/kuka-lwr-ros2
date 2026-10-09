/*
 * joint_state_mapper.h
 *
 * Maps sensor_msgs/JointState by joint name into KDL chain order
 * (guide 8.1: "Map sensor_msgs/JointState by joint name into the KDL
 * chain order. Never assume array order.") and rejects malformed
 * samples per guide 6.1 rather than guessing.
 */

#ifndef ULTRASOUND_TRAJECTORY_JOINT_STATE_MAPPER_H
#define ULTRASOUND_TRAJECTORY_JOINT_STATE_MAPPER_H

#include <cmath>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kdl/jntarray.hpp>

namespace trm {

class JointStateMapper {
public:
    void setClock(rclcpp::Clock::SharedPtr clock) { clock_ = std::move(clock); }
    void configure(const std::vector<std::string>& chain_joint_names) {
        std::lock_guard<std::mutex> lock(mutex_);
        chain_joint_names_ = chain_joint_names;
        q_.resize(static_cast<unsigned int>(chain_joint_names_.size()));
        valid_ = false;
    }

    // Subscriber callback. Copies a coherent, name-mapped snapshot only
    // if the sample passes the guide 6.1 sanity checks; otherwise the
    // previously held snapshot is left untouched and staleness will
    // eventually be caught by the caller's feedback_timeout check.
    void update(const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        if (msg->name.size() != msg->position.size()) {
            markInvalid("joint_state name/position size mismatch");
            return;
        }

        std::unordered_map<std::string, double> by_name;
        by_name.reserve(msg->name.size());
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (!std::isfinite(msg->position[i])) {
                markInvalid("non-finite joint position for " + msg->name[i]);
                return;
            }
            if (!by_name.emplace(msg->name[i], msg->position[i]).second) {
                markInvalid("duplicated joint name " + msg->name[i]);
                return;
            }
        }

        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < chain_joint_names_.size(); ++i) {
            const auto it = by_name.find(chain_joint_names_[i]);
            if (it == by_name.end()) {
                valid_ = false;
                return;
            }
            q_(static_cast<unsigned int>(i)) = it->second;
        }
        // Stamped on receipt, not msg->header.stamp: this class's
        // freshness contract is "how long ago did we last hear
        // something usable", which must not depend on the publisher's
        // clock/header discipline.
        stamp_ = clock_->now();
        valid_ = true;
    }

    // Returns false if no valid sample has ever been mapped. Staleness
    // against feedback_timeout is the caller's responsibility (it owns
    // the TrmLimits, this class does not).
    bool snapshot(KDL::JntArray& q_out, rclcpp::Time& stamp_out) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!valid_) return false;
        q_out = q_;
        stamp_out = stamp_;
        return true;
    }

private:
    void markInvalid(const std::string& /*reason*/) {
        std::lock_guard<std::mutex> lock(mutex_);
        valid_ = false;
    }

    rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};
    mutable std::mutex mutex_;
    std::vector<std::string> chain_joint_names_;
    KDL::JntArray q_;
    rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
    bool valid_ = false;
};

}  // namespace trm

#endif  // ULTRASOUND_TRAJECTORY_JOINT_STATE_MAPPER_H
