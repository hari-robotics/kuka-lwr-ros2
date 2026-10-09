#ifndef LWR_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__CARTESIAN_IMPEDANCE_CONTROLLER_H

// ROS added
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lwr_controllers/msg/stiffness.hpp>
#include <lwr_controllers/msg/cartesian_impedance_point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <realtime_tools/realtime_publisher.h>

// KDL added
#include <kdl/stiffness.hpp>
#include <kdl/trajectory.hpp>
#include <lwr_controllers/ros2_adapter.h>

// BOOST added
#include <boost/scoped_ptr.hpp>

// Base class with useful URDF parsing and kdl chain generator
#include "lwr_controllers/KinematicChainControllerBase.h"

// The format of the command specification
#include "lwr_controllers/srv/set_cartesian_impedance_command.hpp"

namespace lwr_controllers
{
    class CartesianImpedanceController: public lwr_controllers::ControllerAdapter<lwr_controllers::CartesianInterface>
	{
	public:
		CartesianImpedanceController();
		~CartesianImpedanceController();

        bool configure(lwr_controllers::NodeParameters &n);
		controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        void starting(const rclcpp::Time& time);
		void stopping(const rclcpp::Time& time);
		void update_command(const rclcpp::Time& time, const rclcpp::Duration& period);
        void command(const lwr_controllers::msg::CartesianImpedancePoint::ConstSharedPtr &msg);
        bool command_cb(lwr_controllers::srv::SetCartesianImpedanceCommand::Request &req, lwr_controllers::srv::SetCartesianImpedanceCommand::Response &res);
        void updateFT(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &msg);

    protected:

        std::string robot_namespace_, root_name_, tip_name_;
        std::vector<std::string> cart_12_names_, cart_6_names_;
        
        std::vector<JointHandle> cart_handles_;
        bool publish_cartesian_pose_;
		
        // ROS API (topic, service and dynamic reconfigure)
		rclcpp::SubscriptionBase::SharedPtr sub_command_;
        rclcpp::Service<lwr_controllers::srv::SetCartesianImpedanceCommand>::SharedPtr srv_command_;
        rclcpp::SubscriptionBase::SharedPtr sub_ft_measures_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
        std::shared_ptr< realtime_tools::RealtimePublisher< geometry_msgs::msg::PoseStamped > > realtime_pose_pub_;
        
        // Transformation from robot base to controller base
        KDL::Frame robotBase_controllerBase_;

		// Cartesian vars
        KDL::Frame x_ref_;
        KDL::Frame x_des_;
        KDL::Frame x_cur_;
        KDL::Frame x_FRI_;
		KDL::Twist x_err_;
        
        // FRI vars
        std::vector<double> cur_T_FRI_;

		// The jacobian at q_msg
        // KDL::Jacobian J_;

        // Measured external force
        KDL::Wrench f_cur_;
        KDL::Wrench f_des_;
        // KDL::Wrench f_error_;

		// That is, desired kx, ky, kz, krx, kry, krz, they need to be expressed in the same ref as x, and J
		KDL::Stiffness k_des_;
        KDL::Stiffness d_des_;

		// This class does not exist, should we ask for it? 0, for now.
		// KDL::Damping d_des_; 

		// Solver to compute x_cur and x_des
        // boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

		// Solver to compute the Jacobian at q_msr
        // boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

		// Computed torque
        // KDL::JntArray tau_cmd_;

        // Because of the lack of the Jacobian transpose in KDL
        void multiplyJacobian(const KDL::Jacobian& jac, const KDL::Wrench& src, KDL::JntArray& dest);
        
        // Utility function to get the current pose
        void getCurrentPose(KDL::Frame& f);
        
        // Utility function to publish the current pose
        void publishCurrentPose(const KDL::Frame& f);
        
        // Utility function to forward commands to FRI
        void forwardCmdFRI(const KDL::Frame& f);

        // FRI<->KDL conversion
        void fromKDLtoFRI(const KDL::Frame& in, std::vector<double>& out);
        void fromKDLtoFRI(const KDL::Stiffness& in, std::vector<double>& out);
        void fromKDLtoFRI(const KDL::Wrench& in, std::vector<double>& out);
        void fromFRItoKDL(const std::vector<double>& in, KDL::Frame& out);
        void fromFRItoKDL(const std::vector<double>& in, KDL::Stiffness& out);
        void fromFRItoKDL(const std::vector<double>& in, KDL::Wrench& out);
	};

}

#endif
