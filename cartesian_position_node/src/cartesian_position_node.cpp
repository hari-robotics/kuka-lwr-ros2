#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <lwr_controllers/msg/pose_rpy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

class CartesianPositionNode : public rclcpp::Node
{
public:
  explicit CartesianPositionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("cartesian_position_node", options)
  {
    rcl_interfaces::msg::ParameterDescriptor configuration;
    configuration.read_only = true;
    root_ = declare_parameter<std::string>("root_link", "lwr_base_link", configuration);
    end_effector_ = declare_parameter<std::string>("end_effector_link", "lwr_7_link", configuration);
    tooltip_ = declare_parameter<std::string>("tooltip_link", "tcp", configuration);
    alpha_ = declare_parameter<double>("velocity_filter_alpha", 0.7, configuration);
    if (!std::isfinite(alpha_) || alpha_ < 0.0 || alpha_ > 1.0) {
      throw std::invalid_argument("velocity_filter_alpha must be between 0 and 1");
    }
    const auto joint_topic = declare_parameter<std::string>("joint_states_topic", "/lwr/joint_states", configuration);
    const auto description_topic = declare_parameter<std::string>("robot_description_topic", "/robot_description", configuration);
    const auto description = declare_parameter<std::string>("robot_description", "", configuration);
    ee_pose_pub_ = create_publisher<lwr_controllers::msg::PoseRPY>("/cartesian_position/pose/end_effector", 10);
    tool_pose_pub_ = create_publisher<lwr_controllers::msg::PoseRPY>("/cartesian_position/pose/tooltip", 10);
    ee_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cartesian_position/vel/end_effector", 10);
    tool_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cartesian_position/vel/tooltip", 10);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_topic, rclcpp::SensorDataQoS(),
      std::bind(&CartesianPositionNode::on_joints, this, std::placeholders::_1));
    if (!description.empty()) {
      load_description(description);
    } else {
      description_sub_ = create_subscription<std_msgs::msg::String>(
        description_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          if (msg->data == description_) {return;}
          try {
            load_description(msg->data);
          } catch (const std::exception & error) {
            RCLCPP_ERROR(get_logger(), "Cannot load robot description: %s", error.what());
          }
        });
      RCLCPP_INFO(get_logger(), "Waiting for robot description on %s", description_topic.c_str());
    }
  }

private:
  static std::vector<std::string> joint_names(const KDL::Chain & chain)
  {
    std::vector<std::string> names;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
      const auto & joint = chain.getSegment(i).getJoint();
      if (joint.getType() != KDL::Joint::None) {names.push_back(joint.getName());}
    }
    return names;
  }

  void load_description(const std::string & description)
  {
    KDL::Tree tree;
    KDL::Chain ee_chain, tool_chain;
    if (!kdl_parser::treeFromString(description, tree) ||
      !tree.getChain(root_, end_effector_, ee_chain) ||
      !tree.getChain(root_, tooltip_, tool_chain))
    {
      throw std::invalid_argument("URDF must contain chains from root_link to end_effector_link and tooltip_link");
    }
    auto names = joint_names(ee_chain);
    if (names.empty() || names != joint_names(tool_chain)) {
      throw std::invalid_argument("The tooltip must share the end effector's actuated joints through a fixed attachment");
    }
    ee_chain_ = ee_chain;
    tool_chain_ = tool_chain;
    names_ = std::move(names);
    description_ = description;
    filtered_ = false;
    ready_ = true;
    RCLCPP_INFO(get_logger(), "Loaded %zu joints: %s -> %s / %s", names_.size(),
      root_.c_str(), end_effector_.c_str(), tooltip_.c_str());
  }

  static lwr_controllers::msg::PoseRPY pose(const KDL::Frame & frame)
  {
    lwr_controllers::msg::PoseRPY msg;
    msg.position.x = frame.p.x();
    msg.position.y = frame.p.y();
    msg.position.z = frame.p.z();
    frame.M.GetRPY(msg.orientation.roll, msg.orientation.pitch, msg.orientation.yaw);
    return msg;
  }

  static geometry_msgs::msg::Twist twist(const KDL::Twist & velocity)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = velocity.vel.x();
    msg.linear.y = velocity.vel.y();
    msg.linear.z = velocity.vel.z();
    msg.angular.x = velocity.rot.x();
    msg.angular.y = velocity.rot.y();
    msg.angular.z = velocity.rot.z();
    return msg;
  }

  void on_joints(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    if (!ready_) {return;}
    if (msg->name.size() != msg->position.size() ||
      (!msg->velocity.empty() && msg->velocity.size() != msg->name.size()))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring JointState with mismatched array lengths");
      return;
    }
    std::unordered_map<std::string, size_t> indices;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (!indices.emplace(msg->name[i], i).second) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring JointState with duplicate joint names");
        return;
      }
    }
    KDL::JntArray q(names_.size()), q_dot(names_.size());
    const bool has_velocity = !msg->velocity.empty();
    for (size_t i = 0; i < names_.size(); ++i) {
      const auto found = indices.find(names_[i]);
      if (found == indices.end() || !std::isfinite(msg->position[found->second]) ||
        (has_velocity && !std::isfinite(msg->velocity[found->second])))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring JointState with missing or non-finite arm joint values");
        return;
      }
      q(i) = msg->position[found->second];
      if (has_velocity) {q_dot(i) = msg->velocity[found->second];}
    }
    KDL::Frame ee, tool;
    KDL::ChainFkSolverPos_recursive ee_solver(ee_chain_), tool_solver(tool_chain_);
    if (ee_solver.JntToCart(q, ee) < 0 || tool_solver.JntToCart(q, tool) < 0) {
      RCLCPP_ERROR(get_logger(), "Forward position kinematics failed");
      return;
    }
    ee_pose_pub_->publish(pose(ee));
    tool_pose_pub_->publish(pose(tool));
    if (!has_velocity) {
      filtered_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "JointState has no velocity: publishing poses only");
      return;
    }
    KDL::FrameVel ee_velocity, tool_velocity;
    KDL::ChainFkSolverVel_recursive ee_vel_solver(ee_chain_), tool_vel_solver(tool_chain_);
    const KDL::JntArrayVel state(q, q_dot);
    if (ee_vel_solver.JntToCart(state, ee_velocity) < 0 ||
      tool_vel_solver.JntToCart(state, tool_velocity) < 0)
    {
      RCLCPP_ERROR(get_logger(), "Forward velocity kinematics failed");
      return;
    }
    ee_vel_pub_->publish(twist(ee_velocity.GetTwist()));
    auto tool_twist = twist(tool_velocity.GetTwist());
    if (filtered_) {
      tool_twist.linear.x = alpha_ * tool_twist.linear.x + (1 - alpha_) * previous_.linear.x;
      tool_twist.linear.y = alpha_ * tool_twist.linear.y + (1 - alpha_) * previous_.linear.y;
      tool_twist.linear.z = alpha_ * tool_twist.linear.z + (1 - alpha_) * previous_.linear.z;
    }
    previous_ = tool_twist;
    filtered_ = true;
    tool_vel_pub_->publish(tool_twist);
  }

  std::string root_, end_effector_, tooltip_, description_;
  double alpha_;
  bool ready_{false}, filtered_{false};
  KDL::Chain ee_chain_, tool_chain_;
  std::vector<std::string> names_;
  geometry_msgs::msg::Twist previous_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Publisher<lwr_controllers::msg::PoseRPY>::SharedPtr ee_pose_pub_, tool_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ee_vel_pub_, tool_vel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<CartesianPositionNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("cartesian_position_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
