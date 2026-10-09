#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <lwr_controllers/joint_impedance_controller.h>

namespace lwr_controllers {

JointImpedanceController::JointImpedanceController() {}

JointImpedanceController::~JointImpedanceController() {}

bool JointImpedanceController::configure(lwr_controllers::NodeParameters &n)
{
    if (!KinematicChainControllerBase<lwr_controllers::EffortInterface>::configure(n)) return false;
    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());   
    q_des_.resize(kdl_chain_.getNrOfJoints());
    tau_des_.resize(kdl_chain_.getNrOfJoints());
 
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        tau_des_(i) = 0.0;
        K_(i) = 300.0;
        D_(i) = 0.7;
        q_des_(i) = 0.0;
    }

    RCLCPP_DEBUG(nh_.node()->get_logger(), " Number of joints in handle = %lu", joint_handles_.size() );

    for (int i = 0; i < joint_handles_.size(); ++i) {
        if ( !nh_.getParam("stiffness_gains", K_(i) ) ) {
            RCLCPP_WARN(nh_.node()->get_logger(), "Stiffness gain not set in yaml file, Using %f", K_(i));
        }
    }
    for (int i = 0; i < joint_handles_.size(); ++i) {
        if ( !nh_.getParam("damping_gains", D_(i)) ) {
            RCLCPP_WARN(nh_.node()->get_logger(), "Damping gain not set in yaml file, Using %f", D_(i));
        }
    }

    sub_stiffness_ = nh_.subscribe_callback<std_msgs::msg::Float64MultiArray>("stiffness", [this](auto msg){setParam(msg, &K_, "K");});
    sub_damping_ = nh_.subscribe_callback<std_msgs::msg::Float64MultiArray>("damping", [this](auto msg){setParam(msg, &D_, "D");});
    sub_add_torque_ = nh_.subscribe_callback<std_msgs::msg::Float64MultiArray>("additional_torque", [this](auto msg){setParam(msg, &tau_des_, "AddTorque");});
    sub_posture_ = nh_.subscribe("command", 1, &JointImpedanceController::command, this);

    return true;


}

controller_interface::InterfaceConfiguration JointImpedanceController::command_interface_configuration() const
{
    auto config = KinematicChainControllerBase<EffortInterface>::command_interface_configuration();
    for (const auto &joint : joint_names_)
        for (const auto &interface : {"stiffness", "damping", "set_point"}) config.names.push_back(joint + "/" + interface);
    return config;
}

void JointImpedanceController::starting(const rclcpp::Time& time)
{
    joint_stiffness_handles_.clear();joint_damping_handles_.clear();joint_set_point_handles_.clear();
    for (const auto &name : joint_names_) {
        JointHandle k{name},d{name},q{name};
        k.command=find_command(name+"/stiffness");d.command=find_command(name+"/damping");q.command=find_command(name+"/set_point");
        joint_stiffness_handles_.push_back(k);joint_damping_handles_.push_back(d);joint_set_point_handles_.push_back(q);
    }
    // Initializing stiffness, damping, ext_torque and set point values
    for (size_t i = 0; i < joint_handles_.size(); i++) {
        tau_des_(i) = 0.0;
        q_des_(i) = joint_handles_[i].getPosition();
    }


}

void JointImpedanceController::update_command(const rclcpp::Time& time, const rclcpp::Duration& period)
{

    //Compute control law. This controller sets all variables for the JointImpedance Interface from kuka
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        joint_handles_[i].setCommand(tau_des_(i));
        joint_stiffness_handles_[i].setCommand(K_(i));
        joint_damping_handles_[i].setCommand(D_(i));
        joint_set_point_handles_[i].setCommand(q_des_(i));
    }

}


void JointImpedanceController::command(const std_msgs::msg::Float64MultiArray::ConstSharedPtr &msg) {
    if (msg->data.size() == 0) {
        RCLCPP_INFO(nh_.node()->get_logger(), "Desired configuration must be: %lu dimension", joint_handles_.size());
    }
    else if ((int)msg->data.size() != joint_handles_.size()) {
        RCLCPP_ERROR(nh_.node()->get_logger(), "Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
    else
    {
        for (unsigned int j = 0; j < joint_handles_.size(); ++j)
            q_des_(j) = msg->data[j];
    }

}

void JointImpedanceController::setParam(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg, KDL::JntArray* array, std::string s)
{
    if (msg->data.size() == joint_handles_.size())
    {
        for (unsigned int i = 0; i < joint_handles_.size(); ++i)
        {
            (*array)(i) = msg->data[i];
        }
    }
    else
    {
        RCLCPP_INFO(nh_.node()->get_logger(), "Num of Joint handles = %lu", joint_handles_.size());
    }

    RCLCPP_INFO(nh_.node()->get_logger(), "Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

    RCLCPP_INFO(nh_.node()->get_logger(), "New param %s: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", s.c_str(),
             (*array)(0), (*array)(1), (*array)(2), (*array)(3), (*array)(4), (*array)(5), (*array)(6));
}

} // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::JointImpedanceController, controller_interface::ControllerInterface)
