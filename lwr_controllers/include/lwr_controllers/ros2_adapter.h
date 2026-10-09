#ifndef LWR_CONTROLLERS_ROS2_ADAPTER_H
#define LWR_CONTROLLERS_ROS2_ADAPTER_H
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <kdl/frames.hpp>
#include <mutex>
#include <functional>

namespace lwr_controllers {
// Keep the original parameter/topic call sites while using native ROS 2 objects.
class NodeParameters {
public:
  NodeParameters() = default;
  explicit NodeParameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                          std::mutex *mutex = nullptr, std::string prefix = "")
    : node_(std::move(node)), mutex_(mutex), prefix_(std::move(prefix)) {}
  NodeParameters(const NodeParameters &other, const std::string &prefix)
    : node_(other.node_), mutex_(other.mutex_), prefix_(other.prefix_ + prefix + ".") {}
  auto node() const {return node_;}
  std::string getNamespace() const {return node_->get_namespace();}
  std::string resolveName(const std::string &name) const {
    return name.empty() || name.front() == '/' || name.front() == '~' ? name : "~/" + name;
  }
  template<typename T> bool getParam(const std::string &key, T &value) const {
    const auto name = prefix_ + key;
    if (!node_->has_parameter(name)) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.dynamic_typing=true;
      node_->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
    }
    auto parameter = node_->get_parameter(name);
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return false;
    if constexpr(std::is_same_v<T,double>) {
      if(parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value=static_cast<double>(parameter.as_int()); return true;
      }
    }
    value=parameter.get_value<T>(); return true;
  }
  template<typename M> typename rclcpp::Publisher<M>::SharedPtr advertise(const std::string &topic, size_t depth) {
    return rclcpp::create_publisher<M>(node_, resolveName(topic), rclcpp::QoS(std::max<size_t>(1,depth)));
  }
  template<typename M,typename C> rclcpp::SubscriptionBase::SharedPtr subscribe(
      const std::string &topic, size_t depth,
      void(C::*cb)(const std::shared_ptr<const M>&), C *owner) {
    return node_->create_subscription<M>(resolveName(topic), rclcpp::QoS(std::max<size_t>(1,depth)),
      [this,owner,cb](const std::shared_ptr<const M> msg) {
        std::unique_lock<std::mutex> lock;
        if(mutex_) lock=std::unique_lock<std::mutex>(*mutex_);
        (owner->*cb)(msg);
      });
  }
  template<typename M,typename F> rclcpp::SubscriptionBase::SharedPtr subscribe_callback(
      const std::string &topic, F callback) {
    return node_->create_subscription<M>(resolveName(topic),1,
      [this,callback](const std::shared_ptr<const M> msg) {
        std::unique_lock<std::mutex> lock;
        if(mutex_) lock=std::unique_lock<std::mutex>(*mutex_);
        callback(msg);
      });
  }
private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::mutex *mutex_ = nullptr;
  std::string prefix_;
};
inline void wrenchKDLToMsg(const KDL::Wrench &w, geometry_msgs::msg::Wrench &msg) {
  msg.force.x=w.force.x();msg.force.y=w.force.y();msg.force.z=w.force.z();
  msg.torque.x=w.torque.x();msg.torque.y=w.torque.y();msg.torque.z=w.torque.z();
}
inline void wrenchMsgToKDL(const geometry_msgs::msg::Wrench &msg, KDL::Wrench &w) {
  w=KDL::Wrench(KDL::Vector(msg.force.x,msg.force.y,msg.force.z),KDL::Vector(msg.torque.x,msg.torque.y,msg.torque.z));
}
inline void poseMsgToKDL(const geometry_msgs::msg::Pose &msg,KDL::Frame &f) {
  f=KDL::Frame(KDL::Rotation::Quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w),
               KDL::Vector(msg.position.x,msg.position.y,msg.position.z));
}
inline void poseKDLToMsg(const KDL::Frame &f,geometry_msgs::msg::Pose &msg) {
  msg.position.x=f.p.x();msg.position.y=f.p.y();msg.position.z=f.p.z();
  f.M.GetQuaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
}
}
#endif
