#include <math.h>
#include <pluginlib/class_list_macros.hpp>
#include <lwr_controllers/ros2_adapter.h>
#include <tf2_kdl/tf2_kdl.hpp>

#include "lwr_controllers/cartesian_impedance_controller.h"

namespace lwr_controllers 
{
    CartesianImpedanceController::CartesianImpedanceController() : cur_T_FRI_(12) {}
    CartesianImpedanceController::~CartesianImpedanceController() {}

    bool CartesianImpedanceController::configure(lwr_controllers::NodeParameters &n)
    {
        RCLCPP_INFO(nh_.node()->get_logger(), "THIS CONTROLLER USES THE TCP INFORMATION SET ON THE FRI SIDE... SO BE SURE YOU KNOW WHAT YOU ARE DOING!");

        nh_ = n;
        cart_12_names_.clear();
        cart_6_names_.clear();
        cart_handles_.clear();
        publish_cartesian_pose_ = false;
        robotBase_controllerBase_ = KDL::Frame::Identity();
        if (!n.getParam("robot_name", robot_namespace_))
        {
            RCLCPP_WARN_STREAM(nh_.node()->get_logger(), "CartesianImpedanceController: Could not read robot name from parameter server ("<<n.getNamespace()<<"/robot_name), using the namespace...");
            robot_namespace_ = n.getNamespace();
            //return false;
        }
        if (!n.getParam("root_name", root_name_))
        {
            RCLCPP_WARN_STREAM(nh_.node()->get_logger(), "CartesianImpedanceController: Could not read robot root name from parameter server ("<<n.getNamespace()<<"/root_name). Considering it to be equal to #BASE as defined in the KRC!!!");
            root_name_ = robot_namespace_ + "_base_link";
        }
        else
        {
            if (root_name_ != robot_namespace_ + "_base_link") {
                tf2_ros::Buffer buffer(n.node()->get_clock());
                tf2_ros::TransformListener listener(buffer);
                try {
                    auto transform=buffer.lookupTransform(robot_namespace_+"_base_link", root_name_, tf2::TimePointZero, tf2::durationFromSec(1.0));
                    geometry_msgs::msg::Pose pose;
                    pose.position.x=transform.transform.translation.x;
                    pose.position.y=transform.transform.translation.y;
                    pose.position.z=transform.transform.translation.z;
                    pose.orientation=transform.transform.rotation;
                    poseMsgToKDL(pose,robotBase_controllerBase_);
                } catch (const tf2::TransformException &ex) {
                    RCLCPP_ERROR(n.node()->get_logger(),"Cannot resolve controller frame: %s",ex.what());return false;
                }
            }
        }
        if (n.getParam("tip_name", tip_name_))
        {
            RCLCPP_WARN_STREAM(nh_.node()->get_logger(), "CartesianImpedanceController: robot tip name is " << tip_name_ << ". In this controller it is ignored. Using tip name defined in #TOOL in the KRC!!!");
        }
        if (n.getParam("publish_cartesian_pose", publish_cartesian_pose_) && publish_cartesian_pose_)
        {
            RCLCPP_WARN_STREAM(nh_.node()->get_logger(), "Publishing the cartesian position of the robot tip (the #TOOL defined in the KRC!!!)");
        }

        joint_names_.push_back( robot_namespace_ + std::string("_a1_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_a2_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_e1_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_a3_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_a4_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_a5_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_a6_joint") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xx") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yx") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zx") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_pos_x") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xy") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yy") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zy") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_pos_y") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xz") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yz") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zz") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_pos_z") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_X") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_Y") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_Z") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_A") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_B") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_C") );

        // now get all handles, 12 for cart pos, 6 for stiff, 6 for damp, 6 for wrench, 6 for joints
        for(int c = 0; c < 30; ++c)
        {
            if(c < 12)
                cart_handles_.push_back(JointHandle{cart_12_names_.at(c)});
            if(c > 11 && c < 18)
                cart_handles_.push_back(JointHandle{cart_6_names_.at(c-12) + std::string("_stiffness")});
            if(c > 17 && c < 24)
                cart_handles_.push_back(JointHandle{cart_6_names_.at(c-18) + std::string("_damping")});
            if(c > 23 && c < 30)
                cart_handles_.push_back(JointHandle{cart_6_names_.at(c-24) + std::string("_wrench")});
        }
        for(int j = 0; j < 7; ++j)
        {
            joint_handles_.push_back(JointHandle{joint_names_.at(j)});
        }

        sub_command_ = n.subscribe(n.resolveName("command"), 1, &CartesianImpedanceController::command, this);
        srv_command_ = n.node()->create_service<lwr_controllers::srv::SetCartesianImpedanceCommand>(n.resolveName("set_command"),
            [this](const std::shared_ptr<lwr_controllers::srv::SetCartesianImpedanceCommand::Request> request, std::shared_ptr<lwr_controllers::srv::SetCartesianImpedanceCommand::Response> response){std::lock_guard<std::mutex> lock(callback_mutex_); command_cb(*request,*response);});
        sub_ft_measures_ = n.subscribe(n.resolveName("ft_measures"), 1, &CartesianImpedanceController::updateFT, this);
        pub_goal_ = n.advertise<geometry_msgs::msg::PoseStamped>(n.resolveName("goal"),0);
        if (publish_cartesian_pose_)
        {
            realtime_pose_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>(n.advertise<geometry_msgs::msg::PoseStamped>("cartesian_pose", 4)));
        }

        // // may be needed, to set initial commands asap
        // starting(nh_.node()->now());

        return true;
    }

    controller_interface::InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config{controller_interface::interface_configuration_type::INDIVIDUAL,{}};
        for (const auto &handle:cart_handles_) config.names.push_back(handle.name+"/position");
        return config;
    }
    controller_interface::InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const {
        auto config=ControllerAdapter<CartesianInterface>::state_interface_configuration();
        for (const auto &handle:cart_handles_) config.names.push_back(handle.name+"/position");
        return config;
    }
    controller_interface::CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State &) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        try {
            for (auto &handle:cart_handles_) {
                handle.position=find_state(handle.name+"/position");handle.command=find_command(handle.name+"/position");
            }
            starting(get_node()->now());
            return controller_interface::CallbackReturn::SUCCESS;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_node()->get_logger(),"Cartesian FRI interfaces unavailable: %s",e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    void CartesianImpedanceController::starting(const rclcpp::Time& time)
    {
        getCurrentPose(x_ref_);
        x_des_ = x_ref_;

        // Initial Cartesian stiffness
        KDL::Stiffness k( 800.0, 800.0, 800.0, 50.0, 50.0, 50.0 );
        k_des_ = k;

        // Initial Cartesian damping
        KDL::Stiffness d( 0.8, 0.8, 0.8, 0.8, 0.8, 0.8 );
        d_des_ = d;
        
        // Initial force/torque measure
        KDL::Wrench w(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
        f_des_ = w;

        // forward initial commands to hwi
        forwardCmdFRI(x_des_);
    }

    void CartesianImpedanceController::command(const lwr_controllers::msg::CartesianImpedancePoint::ConstSharedPtr &msg)
    {
        // Validate command, for now, only check non-zero of stiffness, damping, and orientation


        // Compute a KDL frame out of the message
        lwr_controllers::poseMsgToKDL( msg->x_fri, x_des_ );
        // Pre-multiply the pose to make sure it is expressed in the correct frame
        x_des_ = robotBase_controllerBase_ * x_des_;

        // Convert Wrench msg to KDL wrench
        lwr_controllers::wrenchMsgToKDL( msg->f_fri, f_des_ );

        // Convert from Stiffness msg array to KDL stiffness
        //if(!(msg->k_fri.x + msg->k_fri.y + msg->k_fri.z + msg->k_fri.rx + msg->k_fri.ry + msg->k_fri.rz == 0.0))
        //{
            RCLCPP_INFO(nh_.node()->get_logger(), "Updating Stiffness command");
            KDL::Stiffness k( msg->k_fri.x, msg->k_fri.y, msg->k_fri.z, msg->k_fri.rx, msg->k_fri.ry, msg->k_fri.rz );
            k_des_ = k;
            
            RCLCPP_INFO(nh_.node()->get_logger(), "Updating Damping command");
            KDL::Stiffness d( msg->d_fri.x, msg->d_fri.y, msg->d_fri.z, msg->d_fri.rx, msg->d_fri.ry, msg->d_fri.rz );
            d_des_ = d;
        //}

        // publishe goal
        geometry_msgs::msg::PoseStamped goal;
        goal.header = msg->header;
        goal.pose = msg->x_fri;
        pub_goal_->publish(goal);

    }

    bool CartesianImpedanceController::command_cb(lwr_controllers::srv::SetCartesianImpedanceCommand::Request &req, lwr_controllers::srv::SetCartesianImpedanceCommand::Response &res)
    {
        auto msg=std::make_shared<lwr_controllers::msg::CartesianImpedancePoint>(req.command);
        command(msg);
        return true;
    }

    void CartesianImpedanceController::updateFT(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &msg)
    {
        // Convert Wrench msg to KDL wrench
        geometry_msgs::msg::Wrench f_meas = msg->wrench;
        lwr_controllers::wrenchMsgToKDL( f_meas, f_cur_);
    }

    void CartesianImpedanceController::update_command(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // get current values
        // std::cout << "Update current values" << std::endl;
        getCurrentPose(x_cur_);
        
        if (publish_cartesian_pose_)
        {
            publishCurrentPose(x_cur_);
        }

        // forward commands to hwi
        // std::cout << "Before forwarding the command" << std::endl;
        forwardCmdFRI(x_des_);
    }

    void CartesianImpedanceController::stopping(const rclcpp::Time& time)
    {

    }

    void CartesianImpedanceController::multiplyJacobian(const KDL::Jacobian& jac, const KDL::Wrench& src, KDL::JntArray& dest)
    {
        Eigen::Matrix<double,6,1> w;
        w(0) = src.force(0);
        w(1) = src.force(1);
        w(2) = src.force(2);
        w(3) = src.torque(0);
        w(4) = src.torque(1);
        w(5) = src.torque(2);

        Eigen::MatrixXd j(jac.rows(), jac.columns());
        j = jac.data;
        j.transposeInPlace();

        Eigen::VectorXd t(jac.columns());
        t = j*w;

        dest.resize(jac.columns());
        for (unsigned i=0; i<jac.columns(); i++)
        dest(i) = t(i);
    }

    void CartesianImpedanceController::fromKDLtoFRI(const KDL::Frame& in, std::vector<double>& out)
    {
        assert(out.size() == 12);
        out[0] = in.M.UnitX().x();
        out[1] = in.M.UnitY().x();
        out[2] = in.M.UnitZ().x();
        out[3] = in.p.x();
        out[4] = in.M.UnitX().y();
        out[5] = in.M.UnitY().y();
        out[6] = in.M.UnitZ().y();
        out[7] = in.p.y();
        out[8] = in.M.UnitX().z();
        out[9] = in.M.UnitY().z();
        out[10] = in.M.UnitZ().z();
        out[11] = in.p.z();

    }

    void CartesianImpedanceController::fromKDLtoFRI(const KDL::Stiffness& in, std::vector<double>& out)
    {
        assert(out.size() == 6);
        for( int d = 0; d < 6; ++d)
        {
            out[d] = in[d];
        }
    }

    void CartesianImpedanceController::fromKDLtoFRI(const KDL::Wrench& in, std::vector<double>& out)
    {
        assert(out.size() == 6);
        out[0] = in.force.x();
        out[1] = in.force.y();
        out[2] = in.force.z();
        out[3] = in.torque.x();
        out[4] = in.torque.y();
        out[5] = in.torque.z();
    }

    void CartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Frame& out)
    {
        assert(in.size() == 12);
        KDL::Rotation R(in[0], in[1], in[2], in[4], in[5], in[6], in[8], in[9], in[10]);
        KDL::Vector p(in[3], in[7], in[11]);
        KDL::Frame T(R, p);
        out = T;
    }

    void CartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Stiffness& out)
    {
        assert(in.size() == 6);
        KDL::Stiffness s(in[0], in[1], in[2], in[3], in[4], in[5]);
        out = s;
    }

    void CartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Wrench& out)
    {
        assert(in.size() == 6);
        KDL::Wrench w(KDL::Vector(in[0], in[1], in[2]), KDL::Vector(in[3], in[4], in[5]));
        out = w;
    }

    void CartesianImpedanceController::forwardCmdFRI(const KDL::Frame& f)
    {
        fromKDLtoFRI(f, cur_T_FRI_);
        for(int c = 0; c < 30; ++c)
        {
            if(c < 12)
                cart_handles_.at(c).setCommand(cur_T_FRI_[c]);
            else if(c < 18)
                cart_handles_.at(c).setCommand(k_des_[c-12]);
            else if(c < 24)
                cart_handles_.at(c).setCommand(d_des_[c-18]);
            else if(c < 30)
                cart_handles_.at(c).setCommand(f_des_[c-24]);
        }
    }

    void CartesianImpedanceController::getCurrentPose(KDL::Frame& f)
    {
        KDL::Rotation cur_R(cart_handles_.at(0).getPosition(),
                            cart_handles_.at(1).getPosition(),
                            cart_handles_.at(2).getPosition(),
                            cart_handles_.at(4).getPosition(),
                            cart_handles_.at(5).getPosition(),
                            cart_handles_.at(6).getPosition(),
                            cart_handles_.at(8).getPosition(),
                            cart_handles_.at(9).getPosition(),
                            cart_handles_.at(10).getPosition());
        KDL::Vector cur_p(cart_handles_.at(3).getPosition(),
                          cart_handles_.at(7).getPosition(),
                          cart_handles_.at(11).getPosition());
        f = KDL::Frame( cur_R, cur_p );
    }
    
    void CartesianImpedanceController::publishCurrentPose(const KDL::Frame& f)
    {
        if (realtime_pose_pub_->trylock()) {
            realtime_pose_pub_->msg_.header.stamp = nh_.node()->now();
            lwr_controllers::poseKDLToMsg(f, realtime_pose_pub_->msg_.pose);
            realtime_pose_pub_->unlockAndPublish();
        }
    }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianImpedanceController, controller_interface::ControllerInterface)
