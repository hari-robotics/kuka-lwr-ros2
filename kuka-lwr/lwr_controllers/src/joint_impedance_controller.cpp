/****** VERSION :  2022-12-23 **********/
/****** MODIFIED : JUNLING FU **********/
/****** FCUNTION : TORQUE CONTROL ******/

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

/****** Added .h file ********/
#include <Eigen/Geometry> //Eigen library
#include <cmath>
#include <lwr_controllers/joint_impedance_controller.h>

namespace lwr_controllers {

JointImpedanceController::JointImpedanceController() {

    RCLCPP_WARN(rclcpp::get_logger("joint_impedance_controller"), "Joint_Impedance_Controller");
    
}

JointImpedanceController::~JointImpedanceController() {}

bool JointImpedanceController::configure(lwr_controllers::NodeParameters &n){
    if (!KinematicChainControllerBase<lwr_controllers::EffortInterface>::configure(n)) return false;
    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());   
    q_des_.resize(kdl_chain_.getNrOfJoints());
    tau_des_.resize(kdl_chain_.getNrOfJoints());
    q_start_.resize(kdl_chain_.getNrOfJoints());

    K_joint.resize(kdl_chain_.getNrOfJoints());
    D_joint.resize(kdl_chain_.getNrOfJoints());
        
    tau_.resize(kdl_chain_.getNrOfJoints());
    tau_N_.resize(kdl_chain_.getNrOfJoints());
    tau_d_.resize(kdl_chain_.getNrOfJoints());
    
    est_ext_trq_.resize(kdl_chain_.getNrOfJoints());
	
//	Set joint stiffness and damping (put all to zero to remove null-space control)   
    K_joint(0) = 0.0; //20 1500
    K_joint(1) = 0.0;
    K_joint(2) = 0.0;
    K_joint(3) = 0.0; 
    K_joint(4) = 0.0;
    K_joint(5) = 0.0;
    K_joint(6) = 0.0;
   
    D_joint(0) = 0.0; //5 77
    D_joint(1) = 0.0;
    D_joint(2) = 0.0;
    D_joint(3) = 0.0;
    D_joint(4) = 0.0;
    D_joint(5) = 0.0;
    D_joint(6) = 0.0;
    
//	Set Cartesian stiffness
    K_cart.resize(6); 
    K_cart(0) = 2500.0; // 500 1000 1500 2500
    K_cart(1) = 2500.0;
    K_cart(2) = 2500.0;	// K_z
    K_cart(3) = 75.0; // 14.0 50 100
    K_cart(4) = 75.0; 
    K_cart(5) = 75.0;
    
//	Set cartesian damping
//TODO: set critical damping from stiffnes with a define (just on z-axis)
    D_cart.resize(6);
    D_cart(0) = 100; // 44.5 63.5 77.5 100
    D_cart(1) = 100;
    D_cart(2) = 100; //	D_z
    D_cart(3) = 0.8;  // 0.7 is ok with stiffness 0.0 
    D_cart(4) = 0.8; 
    D_cart(5) = 0.8;
    
	for (size_t i = 0; i < joint_handles_.size(); i++){	
        // TODO(ros2-migration): ROS 1 read the stiffness/damping/set point and joint handles
        // here (controller load). ros2_control loans the interfaces only at activation: the
        // handle values are not available here and q_start_ is taken at the first starting().
        K_(i) = 300.0;
        D_(i) = 0.7;
        q_start_(i) = joint_handles_[i].getPosition();
        q_des_(i) = 0.0;
        
        tau_des_(i) = joint_handles_[i].getPosition();
        tau_(i) = joint_handles_[i].getEffort();
        
        CartesianForces.resize(6);
        vel_linear_.resize(6);
            
        J_.resize(kdl_chain_.getNrOfJoints());
        M_.resize(kdl_chain_.getNrOfJoints());
            
        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_,gravity_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

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
       
//	Subscribe to /est_ext_trq for estimated external torque from /lwr_hw (ADDED 10.22) 
	sub_est_ext_trq_ = nh_.subscribe_callback<std_msgs::msg::Float64MultiArray>("/est_ext_torque", [this](auto msg){setParam(msg, &est_ext_trq_, "EstExtTrq");});
	sub_stiff = nh_.subscribe("/stiffness", 10, &JointImpedanceController::updateStiffness, this);
	
////////////////	DEBUG	
	pub_vel = nh_.advertise<geometry_msgs::msg::Point>("/velocity", 10);
	pub_vel_jac = nh_.advertise<geometry_msgs::msg::Twist>("/velocity_jac", 10);
	pub_cart_force = nh_.advertise<geometry_msgs::msg::Twist>("/cartesian_forces", 10);
	
    return true;
}

/************************** starting function 2022/12/19 ***************************************/

controller_interface::InterfaceConfiguration JointImpedanceController::command_interface_configuration() const
{
    auto config = KinematicChainControllerBase<EffortInterface>::command_interface_configuration();
    for (const auto &joint : joint_names_)
        for (const auto &interface : {"stiffness", "damping", "set_point"}) config.names.push_back(joint + "/" + interface);
    return config;
}

void JointImpedanceController::starting(const rclcpp::Time& time){
    joint_stiffness_handles_.clear();joint_damping_handles_.clear();joint_set_point_handles_.clear();
    for (const auto &name : joint_names_) {
        JointHandle k{name},d{name},q{name};
        k.command=find_command(name+"/stiffness");d.command=find_command(name+"/damping");q.command=find_command(name+"/set_point");
        joint_stiffness_handles_.push_back(k);joint_damping_handles_.push_back(d);joint_set_point_handles_.push_back(q);
    }
    // ROS 1 read q_start_ in init() (see configure)
    if (!q_start_initialized_) {
        for (size_t i = 0; i < joint_handles_.size(); i++) q_start_(i) = joint_handles_[i].getPosition();
        q_start_initialized_ = true;
    }
//	Initializing stiffness, damping, ext_torque and set point values
    for (size_t i = 0; i < joint_handles_.size(); i++) {
    	q_des_(i) = joint_handles_[i].getPosition();
        tau_des_(i) = 0.0;
        
//	This is joint torque to get the robot to desired position
        tau_(i) = joint_handles_[i].getEffort();
        tau_N_(i) = 0.0;
        tau_d_(i) = 0.0;
    }
     
    for (size_t i = 0; i < joint_handles_.size(); i++){
    	joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    	joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }
    
//	Joints initial positions 
    q0_ = q_start_.data;	//TODO: delete
    
//	Forward kinematics from x to q 
    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_des_);
    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_meas_);
    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_meas_old_);

//	Compute quaternion of desired pose    
    x_des_.M.GetQuaternion(qx_des_, qy_des_, qz_des_, qw_des_);
    orientation_des_.x() = qx_des_; orientation_des_.y() = qy_des_; orientation_des_.z() = qz_des_; orientation_des_.w() = qw_des_;	
    RCLCPP_INFO(nh_.node()->get_logger(), "orientation_des_.x: %f,%f,%f,%f",qx_des_,qy_des_,qz_des_,qw_des_); // Checked, 2022-12-22
}

void JointImpedanceController::updateStiffness(const std_msgs::msg::Float64::ConstSharedPtr& stiff_des_msg){
	RCLCPP_INFO(nh_.node()->get_logger(), "Getting optimized stiffness");
//	K_cart(2) = stiff_des_msg->data;
}


/************************** update function 2022/12/19 JLF**************************/

void JointImpedanceController::update_command(const rclcpp::Time& time, const rclcpp::Duration& period){

/*	
    RCLCPP_INFO(nh_.node()->get_logger(), "stiffness: %f", K_cart(2));
	D_cart(2) = 2*sqrt(K_cart(2));
	RCLCPP_INFO(nh_.node()->get_logger(), "D_z: %f", D_cart(2));	
*/
	
//	JOINT ORDER of joint_msr_states: [lwr_a1_joint, lwr_a2_joint, lwr_e1_joint, lwr_a3_joint, lwr_a4_joint, lwr_a5_joint, lwr_a6_joint]

    for (int i = 0; i < joint_handles_.size(); i++){
    	joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    	joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    	
    	vel_jnt_(i) = joint_handles_[i].getVelocity(); // velocity
    }
        
//	Retrieve Jacobian and Inertia matrix from KDL
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_);
        
        id_solver_->JntToMass(joint_msr_states_.q, M_);
  
//	Compute current cartesian position
	fk_pos_solver_->JntToCart(joint_msr_states_.q,x_meas_);

////////	ORIENTATION CONSTRAINT (EE and TIP orientation are the same)

//	Compute quaternion of measured pose
	x_meas_.M.GetQuaternion(qx_, qy_, qz_, qw_);
	orientation_.x() = qx_; orientation_.y() = qy_; orientation_.z() = qz_; orientation_.w() = qw_;
	
	Matrix3d rot_matrix_;
	rot_matrix_ << x_meas_.M(0,0), x_meas_.M(0,1), x_meas_.M(0,2),
    	 		   x_meas_.M(1,0), x_meas_.M(1,1), x_meas_.M(1,2),
     	 		   x_meas_.M(2,0), x_meas_.M(2,1), x_meas_.M(2,2);
     	 		  
//	Compute the error betwen desired and measured orientation	  
	Eigen::Quaterniond error_quaternion_(orientation_.inverse() * orientation_des_);
 	Eigen::Vector3d error_(error_quaternion_.x(), error_quaternion_.y(), error_quaternion_.z());
	error_ << -rot_matrix_ * error_;

//TODO 	
	if (orientation_des_.coeffs().dot(orientation_.coeffs()) < 0.0){
//		orientation_.coeffs() << -orientation_.coeffs();
		RCLCPP_INFO(nh_.node()->get_logger(), "quaternion error less than zero");
    }
//TODO

////////	Convert J_ and M_ to Eigen::Matrix - to apply Eigen operations
	for (int k=0; k<6; ++k){
		for (int l=0; l<7; ++l){
			J_Eigen_(k,l) = J_.operator() (k, l);
		}
	}
	for (int k=0; k<7; ++k){
		for (int l=0; l<7; ++l){
			M_Eigen_(k,l) = M_.operator() (k, l);
		}
	}
	
//	Compute and publish EE linear velocity
	vel_ee_ = J_Eigen_ * vel_jnt_;
	
	vel_msg_jac.linear.x = vel_ee_(0);	//x
	vel_msg_jac.linear.y = vel_ee_(1);	//y
	vel_msg_jac.linear.z = vel_ee_(2);	//z
	vel_msg_jac.angular.x = vel_ee_(3);	//rx
	vel_msg_jac.angular.y = vel_ee_(4);	//ry
	vel_msg_jac.angular.z = vel_ee_(5);	//rz
	pub_vel_jac->publish(vel_msg_jac);
	
//	Compute and publish EE linear velocity
	vel_msg.x = (x_meas_.p(0) - x_meas_old_.p(0))/period.seconds();
	vel_msg.y = (x_meas_.p(1) - x_meas_old_.p(1))/period.seconds();
	vel_msg.z = (x_meas_.p(2) - x_meas_old_.p(2))/period.seconds();
	pub_vel->publish(vel_msg);
		
		
//	Compute cartesian forces to convert the main task from cartesian to joint space
	for(int i = 0; i < 6; i++) {
		if(i<3){
			//	POSITION
			CartesianForces(i) = K_cart(i)*(x_des_.p(i) - x_meas_.p(i)) - D_cart(i)*(x_meas_.p(i) - x_meas_old_.p(i))/period.seconds();
			// CartesianForces(i) = K_cart(i)*(x_des_.p(i) - x_meas_.p(i)) - D_cart(i)*vel_ee_(i);
			// CartesianForces(i) = K_cart(i)*(x_des_.p(i) - x_meas_.p(i)) - D_cart(i)*((vel_ee_(i)-vel_ee_old_(i))/2);
        }
        else{

			CartesianForces(i) = 0.0;	//	leave only this is the case for position only control.			
			vel_linear_(i) = 0.0;

            //	ORIENTATION VELOCITY METHOD-1
/*			for (int j = 0; j < 7; j++){
				//	compute joints linear velocity
				vel_linear_(i) += (J_(i,j)*joint_msr_states_.qdot(j));
			}
*/
            // ORIENTATION VELOCITY METHOD-2 
            vel_linear_(i) = vel_ee_(i);
            
            // ORIENTATION FORCE
			CartesianForces(i) = (-K_cart(i)*error_(i-3) -D_cart(i)*vel_linear_(i)); // set the default damping value as 0.7
            // CartesianForces(i) = (-K_cart(i)*error_(i-3));
            // CartesianForces(i) = -D_cart(i)*vel_linear_(i);

            RCLCPP_INFO(nh_.node()->get_logger(), "Cartesian_Orientation_Force: %f", CartesianForces(i));

/*			 
			RCLCPP_INFO_STREAM(nh_.node()->get_logger(), " ang. vel. x " << (x_meas_.M(2,1) - x_meas_old_.M(2,1))/period.seconds()); //wx
			RCLCPP_INFO_STREAM(nh_.node()->get_logger(), " ang. vel. y " << (x_meas_.M(0,2) - x_meas_old_.M(0,2))/period.seconds()); //wy
			RCLCPP_INFO_STREAM(nh_.node()->get_logger(), " ang. vel. z " << (x_meas_.M(1,0) - x_meas_old_.M(1,0))/period.seconds()); //wz
*/			
        }
	}
	
	cart_force_msg.linear.x = CartesianForces(0);
	cart_force_msg.linear.y = CartesianForces(1);
	cart_force_msg.linear.z = CartesianForces(2);

	cart_force_msg.angular.x = CartesianForces(3);
	cart_force_msg.angular.y = CartesianForces(4);
	cart_force_msg.angular.z = CartesianForces(5);
	
	pub_cart_force->publish(cart_force_msg); // To verify the calculated force to change position
	

//	Task (desired position) torque: tau_task = J'*F;
	for(size_t i = 0; i < joint_handles_.size(); i++) {
		tau_(i) = 0.0;
		for(size_t j = 0; j < 6; j++) {
			tau_(i) += J_(j,i)*CartesianForces(j);                                   
		}
	}

//	Null-space Projector matrix
	N_ = I_ - J_Eigen_.transpose() * (M_Eigen_.inverse() * J_Eigen_.transpose() * (J_Eigen_ * M_Eigen_.inverse() * J_Eigen_.transpose()).inverse()).transpose();

//	Null-space torque: tau_N = N(q)*(K(q)*(q_des - q_msr) - D(q)*qdot_msr - tau_ext(q));	
	for(size_t i = 0; i < joint_handles_.size(); i++) {
		tau_N_(i) = 0.0;
		for(size_t j = 0; j < joint_handles_.size(); j++) {
			tau_N_(i) += N_.coeff(i,j)*((K_joint(j)*(q_start_(j)-joint_msr_states_.q(j)) - D_joint(j)*joint_msr_states_.qdot(j)) - est_ext_trq_(j));
		}
	}
	
//	Total desired torque = task torque + null-space torque;
	for(size_t i = 0; i < joint_handles_.size(); i++) {
		tau_d_(i) = tau_(i) + tau_N_(i);	// pose control + null-space
		// tau_d_(i) = tau_(i);	// just pose control
	}

//	Compute control law. This controller sets all variables for the JointImpedance Interface from kuka
//  Check the default value of the K_(i), D_(i), and q_des_(i) Junling Fu 2022/12/16 default one from manual is 1000
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
	joint_handles_[i].setCommand(tau_d_(i));
    joint_stiffness_handles_[i].setCommand(0*K_(i)); // It will be the default value if not specified as 0 JLF 2022/12/21
    joint_damping_handles_[i].setCommand(0*D_(i));
	// joint_set_point_handles_[i].setCommand(q_des_(i));	//TODO or q_start ? update this?	   
	// joint_set_point_handles_[i].setCommand(q_start_(i)); // needs to be tested 2022-12-20
	joint_set_point_handles_[i].setCommand(joint_msr_states_.q(i)); // needs to be tested 2022-12-20
       
    RCLCPP_INFO(nh_.node()->get_logger(), "setCommand of the joint %f", joint_msr_states_.q(i));
    // RCLCPP_INFO(nh_.node()->get_logger(), "Set of the q_start %f", q_start_(i));
	// RCLCPP_INFO(nh_.node()->get_logger(), "Stiffness Value %f", 0*K_(i)); 
    // RCLCPP_INFO(nh_.node()->get_logger(), "Damping Value %f", 0*D_(i)); 	

    }
    
    // RCLCPP_INFO(nh_.node()->get_logger(), "q_des %f", q_des_(0));  
    
    x_meas_old_ = x_meas_;
    vel_ee_old_ = vel_ee_;

}

/************************** command function 2022/12/19 JLF **************************/

void JointImpedanceController::command(const std_msgs::msg::Float64MultiArray::ConstSharedPtr &msg) { 
//	THIS FUNCTION HAS BEEN COMPLETELY MODIFIED (10.22)
    if (msg->data.size() == 0) {
        RCLCPP_INFO(nh_.node()->get_logger(), "Desired configuration must be: %lu or 6 dimension ", joint_handles_.size());
        }
    else if ( (int)msg->data.size() != joint_handles_.size() && (int)msg->data.size() != 6 ){
        RCLCPP_ERROR(nh_.node()->get_logger(), "Posture message had the wrong size: %d", (int)msg->data.size());
        return;
        } 
//	If the message published on joint_impedance_controller/command is in joint space (data_size is 7: joint angles) >> put the message inside q_des_ (joint desired configuration)
    else if ( (int)msg->data.size() == joint_handles_.size() ){
            for (unsigned int j = 0; j < joint_handles_.size(); j++)
                q_des_(j) = msg->data[j];
                RCLCPP_INFO(nh_.node()->get_logger(), "Getting joint angles");
        }
//	If the message published on joint_impedance_controller/command is in cartesian space (data_size is 6: XYZRPY) >> put the message inside x_des_ (cartesian desired configuration)            
    else if  ( (int)msg->data.size() == 6 ){
            for (unsigned int j = 0; j < 3; j++){
                x_des_.p(j) = msg->data[j];
				RCLCPP_INFO(nh_.node()->get_logger(), "Getting cartesian position");
            }
    }

}

/************************** Setparameter function 2022/12/19 **************************/

void JointImpedanceController::setParam(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg, KDL::JntArray* array, std::string s){
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

//	RCLCPP_INFO(nh_.node()->get_logger(), "Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

//	RCLCPP_INFO(nh_.node()->get_logger(), "New param %s: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", s.c_str(),
//		(*array)(0), (*array)(1), (*array)(2), (*array)(3), (*array)(4), (*array)(5), (*array)(6));
}

} // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::JointImpedanceController, controller_interface::ControllerInterface)
