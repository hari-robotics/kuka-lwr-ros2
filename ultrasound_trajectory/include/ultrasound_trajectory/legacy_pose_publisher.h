/*
 * legacy_pose_publisher.h
 *
 * Publishes a complete absolute pose to the legacy
 * lwr_controllers/OneTaskInverseKinematics command topic. Guide
 * Section 2/8.2: "Use id=0 only" (id=1 forces identity orientation,
 * id=2 forces zero translation -- either can generate a large
 * unintended move). id=0 is hardcoded here so no caller can ever set
 * it to anything else.
 */

#ifndef ULTRASOUND_TRAJECTORY_LEGACY_POSE_PUBLISHER_H
#define ULTRASOUND_TRAJECTORY_LEGACY_POSE_PUBLISHER_H

#include <kdl/frames.hpp>
#include <lwr_controllers/msg/pose_rpy.hpp>
#include <rclcpp/rclcpp.hpp>

namespace trm {

class LegacyPosePublisher {
public:
    LegacyPosePublisher(rclcpp::Publisher<lwr_controllers::msg::PoseRPY>::SharedPtr pub, bool monitor_only)
        : pub_(pub), monitor_only_(monitor_only) {}

    // Always builds the message (so monitor_only can still be
    // inspected/logged by the caller), but only writes to the wire
    // when monitor_only_ is false -- guide Section 9 step 7: "Launch
    // the wrapper in monitor_only mode ... without publishing
    // commands."
    lwr_controllers::msg::PoseRPY publishFullPose(const KDL::Frame& frame) {
        lwr_controllers::msg::PoseRPY msg;
        msg.id = 0;
        msg.position.x = frame.p.x();
        msg.position.y = frame.p.y();
        msg.position.z = frame.p.z();
        frame.M.GetRPY(msg.orientation.roll, msg.orientation.pitch, msg.orientation.yaw);

        if (!monitor_only_) {
            pub_->publish(msg);
        }
        last_published_ = frame;
        return msg;
    }

    bool monitorOnly() const { return monitor_only_; }
    const KDL::Frame& lastPublished() const { return last_published_; }

private:
    rclcpp::Publisher<lwr_controllers::msg::PoseRPY>::SharedPtr pub_;
    bool monitor_only_;
    KDL::Frame last_published_;
};

}  // namespace trm

#endif  // ULTRASOUND_TRAJECTORY_LEGACY_POSE_PUBLISHER_H
