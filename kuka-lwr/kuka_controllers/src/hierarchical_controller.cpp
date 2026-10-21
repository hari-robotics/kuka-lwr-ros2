#include <iostream>
#include "../include/hierarchical_controller.h"
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <fstream>
#include <math.h>
#include <iostream>
int des_flag=1;

void Jacobian(KDL::JntArray& q,KDL::Jacobian& J,float d1, float d2, float d3)
{
    //update the jacobian matrix of endeffector of tool 
    float s1 = sin(q(0));
    float c1 = cos(q(0));
    float s2 = sin(q(1));
    float c2 = cos(q(1));
    float s3 = sin(q(2));
    float c3 = cos(q(2));
    float s4 = sin(q(3));
    float c4 = cos(q(3));
    float s5 = sin(q(4));
    float c5 = cos(q(4));
    float s6 = sin(q(5));
    float c6 = cos(q(5));
    
    J(0,0) = -d2*(s4*(c1*s3+s1*c2*c3)-s1*c4*s2) - d3*(c6*(s4*(c1*s3+s1*c2*c3)-s1*c4*s2)-s6*(c5*(c4*(c1*s3+s1*c2*c3)+s1*s2*s4)+s5*(c3*c1-s1*c2*s3))) + d1*s1*s2;
    J(0,1) = -d2*(s4*c1*s2*c3+c1*c4*c2) - d3*(c6*(s4*c1*s2*c3+c1*c4*c2)-s6*(c5*(c4*c1*s2*c3-c1*c2*s4)-s5*c1*s2*s3)) - d1*c1*c2;
    J(0,2) = -d2*s4*(c3*s1+c1*c2*s3) - d3*(c6*s4*(c3*s1+c1*c2*s3)-s6*(c5*c4*(c3*s1+c1*c2*s3)+s5*(-s1*s3+c1*c2*c3)));
    J(0,3) = -d2*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4) - d3*(c6*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)-s6*c5*(-s4*(s1*s3-c1*c2*c3)-c1*c4*s2));
    J(0,4) = d3*s6*(-s5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+c5*(c3*s1+c1*c2*s3));
    J(0,5) = -d3*(-s6*(s4*(s1*s3-c1*c2*c3)+c1*c4*s2)-c6*(c5*(c4*(s1*s3-c1*c2*c3)-c1*s2*s4)+s5*(c3*s1+c1*c2*s3)));
    J(0,6) = 0;
    
    J(1,0) = d2*(s4*(-s1*s3+c1*c2*c3)-c1*c4*s2) + d3*(c6*(s4*(-s1*s3+c1*c2*c3)-c1*c4*s2)-s6*(c5*(c4*(-s1*s3+c1*c2*c3)+c1*s2*s4)+s5*(-c3*s1-c1*c2*s3))) - d1*c1*s2;
    J(1,1) = d2*(-s2*s4*c3*s1-c4*s1*c2)+d3*(c6*(-s2*s4*c3*s1-c4*s1*c2)-s6*(c5*(-c4*s2*c3*s1+c2*s1*s4)+s5*s2*s1*s3)) - d1*s1*c2;
    J(1,2) = d2*s4*(c1*c3-c2*s1*s3) + d3*(c6*s4*(c1*c3-c2*s1*s3)-s6*(c5*c4*(c1*c3-c2*s1*s3)+s5*(-c1*s3-c2*c3*s1)));
    J(1,3) = d2*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4) + d3*(c6*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)-s6*c5*(-s4*(c1*s3+c2*c3*s1)+c4*s1*s2));
    J(1,4) = -d3*s6*(-s5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+c5*(c1*c3-c2*s1*s3));
    J(1,5) = d3*(-s6*(s4*(c1*s3+c2*c3*s1)-c4*s1*s2)-c6*(c5*(c4*(c1*s3+c2*c3*s1)+s1*s2*s4)+s5*(c1*c3-c2*s1*s3)));
    J(1,6) = 0;
    
    J(2,0) = 0;
    J(2,1) = d3*(s6*(c5*(-s2*s4-c3*c4*c2)+c2*s3*s5)+c6*(-s2*c4+c3*c2*s4)) + d2*(-s2*c4+c3*c2*s4) - d1*s2;
    J(2,2) = d3*(s6*(c5*s3*c4*s2+s2*c3*s5)-c6*s3*s2*s4) - d2*s3*s2*s4;
    J(2,3) = d3*(s6*c5*(c2*c4+c3*s2*s4)+c6*(-c2*s4+c3*c4*s2)) + d2*(-c2*s4+c3*c4*s2);
    J(2,4) = d3*s6*(-s5*(c2*s4-c3*c4*s2)+s2*s3*c5);
    J(2,5) = d3*(c6*(c5*(c2*s4-c3*c4*s2)+s2*s3*s5)-s6*(c2*c4+c3*s2*s4));
    J(2,6) = 0;
}


void NullSpaceMatrix(KDL::Jacobian& J, Eigen::MatrixXd& J_pinv, Eigen::MatrixXd& N)
    {
	for(int i = 0; i < 7; ++i)
		{for(int j = 0; j < 7; ++j)
		    {N(i,j) = 0;
			for(int k = 0; k < 6; ++k)
		    {
		        N(i,j) -= J_pinv(i,k) * J(k,j);
			
			
		    }if (i==j)N(i,j)+=1;
			//std::cout << N(i,j) << " | ";
		}//std::cout<<std::endl;
	}//std::cout<<std::endl;
}


void DirectionToRotation(KDL::Vector dir, KDL::Rotation& rot)
{
	KDL::Vector up(0,0,1);

	KDL::Vector x = up * dir;
	
	x.Normalize();

	KDL::Vector y = dir * x;
	
	y.Normalize();

	KDL::Rotation rotation = KDL::Rotation(x(0),y(0),dir(0),x(1),y(1),dir(1),x(2),y(2),dir(2));

	rot = rotation;


}


namespace kuka_controllers 
{
    HierarchicalController::HierarchicalController() {}
    HierarchicalController::~HierarchicalController() {}
 
    bool HierarchicalController::configure(lwr_controllers::NodeParameters &n)
    {
        if( !(KinematicChainControllerBase<lwr_controllers::PositionInterface>::configure(n)) )
        {
            RCLCPP_ERROR(nh_.node()->get_logger(), "Couldn't initilize OneTaskInverseKinematics controller.");
            return false;
        }
        // ROS 2: the parameter API has no float type (ROS 1 getParam(float&) also accepted the
        // integer values kd_main: 0 / kd_rcm: 0), so the gains are read as double
        double kp_main, kd_main, kp_rcm, kd_rcm;
        if(!n.getParam("kp_main",kp_main) || !n.getParam("kd_main",kd_main) || !n.getParam("kp_rcm",kp_rcm) || !n.getParam("kd_rcm",kd_rcm))
        { 
           RCLCPP_ERROR_STREAM(nh_.node()->get_logger(), "Controller gains are not set correctly");
           return false;
        }
        Kp1 = kp_main; Kd1 = kd_main; Kp2 = kp_rcm; Kd2 = kd_rcm;
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

        q_cmd_.resize(kdl_chain_.getNrOfJoints());
        J_end_.resize(kdl_chain_.getNrOfJoints());
	J_tooltip_.resize(kdl_chain_.getNrOfJoints());
	N_ = Eigen::MatrixXd(7,7);


        // TODO(ros2-migration): ROS 1 read the joint handles here (controller load) and derived
        // the desired pose, RCM point and tool vector from them. The ros2_control state
        // interfaces are loaned only at activation (a read here returns 0 and the first update
        // would command q = 0), so this block runs once in starting() at the first activation
        // after configure; ROS 1 starting() was empty, later activations do not re-run it.
	
	//x_des_rcm_.p = pose_rcm_;
       
	cmd_flag_ = 0;

        //	errorfile.open(".~/Centro_catkin_ws/errors.txt" );
	//xfile.open(".~/Centro_catkin_ws/x.txt");
	//rcmfile.open("~/Centro_catkin_ws/rcm.txt");
        sub_command_ = nh_.subscribe("command", 1, &HierarchicalController::command, this);

        return true;
    }

    void HierarchicalController::starting(const rclcpp::Time& time)
    {
        if (init_state_read_) return;
        init_state_read_ = true;

        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        //Desired posture is the current one
        
        float tool_length;
        tool_length=0.252;//0.22
        float init_tip[]={};
        init_tip[0]=x_.M(0,2)*tool_length;
        init_tip[1]=x_.M(1,2)*tool_length;
        init_tip[2]=x_.M(2,2)*tool_length;

	x_.p[0] +=init_tip[0];
	x_.p[1] +=init_tip[1];
	x_.p[2] +=init_tip[2];
	pose_rcm_(0) = x_.p[0] - 0.5*init_tip[0];//middle of tooltip
	pose_rcm_(1) = x_.p[1] - 0.5*init_tip[1];
	pose_rcm_(2) = x_.p[2] - 0.5*init_tip[2];
	x_des_ = x_;

        RCLCPP_INFO(nh_.node()->get_logger(), "Init position: %lf,%lf,%lf", x_des_.p[0],x_des_.p[1],x_des_.p[2]);
        RCLCPP_INFO(nh_.node()->get_logger(), "Init X_axis: %lf,%lf,%lf", x_des_.M(0,0),x_des_.M(1,0),x_des_.M(2,0));
        RCLCPP_INFO(nh_.node()->get_logger(), "Init Y_axis: %lf,%lf,%lf", x_des_.M(0,1),x_des_.M(1,1),x_des_.M(2,1));
        RCLCPP_INFO(nh_.node()->get_logger(), "Init Z_axis: %lf,%lf,%lf", x_des_.M(0,2),x_des_.M(1,2),x_des_.M(2,2));
	
	d1 = 0.4; 
	d2 = 0.39; 
	d3 = tool_length+0.078; 
	d4 = 0.078;

	/*Kp1 = 100.0;
	Kp2 = 1000.0;
	Kd1 = 0.0;
	Kd2 = 0.0;*/
        RCLCPP_INFO(nh_.node()->get_logger(), "Controller gains: kp_main = %f, kd_main = %f,kp_rcm = %f,kd_rcm = %f",Kp1,Kd1,Kp2,Kd2);
	x_rcm_err_last_.vel(0) = x_rcm_err_last_.vel(1) = x_rcm_err_last_.vel(2) = 0;
	x_err_last_.vel(0) = x_err_last_.vel(1) = x_err_last_.vel(2) = 0;


	//std::cout << pose_rcm_(0) << " "<<pose_rcm_(1)<<" "<<pose_rcm_(2)<<std::endl;
	//std::cout << x_.M(0,2) << " "<<x_.M(1,2)<<" "<<x_.M(2,2)<<std::endl;
	//std::cout << init_tip[0] << " "<<init_tip[1]<<" "<<init_tip[2]<<std::endl;	//RCM
	float norm = sqrt(pow(init_tip[0]-pose_rcm_(0),2) + pow(init_tip[1]-pose_rcm_(1),2) +pow(init_tip[2]-pose_rcm_(2),2));
	toolvector_(0) = ((x_des_.p[0] - pose_rcm_(0))/norm)*tool_length;
	toolvector_(1) = ((x_des_.p[1] - pose_rcm_(1))/norm)*tool_length;
	toolvector_(2) = ((x_des_.p[2] - pose_rcm_(2))/norm)*tool_length;
    }

    void HierarchicalController::update_command(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
	

        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }

        if (cmd_flag_)
        {

	x_des_.M.GetRPY(end_orientation(0),end_orientation(1),end_orientation(2));
	//ROS_INFO("orientation desired: %f, %f, %f",end_orientation(0),end_orientation(1),end_orientation(2));
           jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_end_);
	   jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_tooltip_);

            // computing Jacobian
	    Jacobian(joint_msr_states_.q, J_tooltip_, d1,d2,d3);
	    //Jacobian(joint_msr_states_.q, J_end_, d1,d2,d4);
	
            // computing J_pinv_
	    pseudo_inverse(J_tooltip_.data, J_tooltip_pinv_);
            pseudo_inverse(J_end_.data, J_end_pinv_);
            

            // computing forward kinematics
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);


	    //x_ is the pose of the end effector i.e. x_rcm_;
	    x_rcm_ = x_;

            float tool_length;
            tool_length=0.275;
              
            float tip[]={};
            tip[0]=x_.M(0,2)*tool_length;
            tip[1]=x_.M(1,2)*tool_length;
            tip[2]=x_.M(2,2)*tool_length;


	    //Actual tooltip pose
	    x_.p[0] +=tip[0];
	    x_.p[1] +=tip[1];
	    x_.p[2] +=tip[2];
	
            

	    //RCM
	    float norm = sqrt(pow(x_des_.p[0]-pose_rcm_(0),2) + pow(x_des_.p[1]-pose_rcm_(1),2) +pow(x_des_.p[2]-pose_rcm_(2),2));
	    toolvector_(0) = ((x_des_.p[0] - pose_rcm_(0))/norm)*tool_length;
	    toolvector_(1) = ((x_des_.p[1] - pose_rcm_(1))/norm)*tool_length;
	    toolvector_(2) = ((x_des_.p[2] - pose_rcm_(2))/norm)*tool_length;


	    x_des_rcm_.p = x_des_.p - toolvector_;

	    NullSpaceMatrix(J_tooltip_, J_tooltip_pinv_, N_);



            // end-effector position error
            x_err_.vel = (x_des_.p - x_.p);
	    x_rcm_err_.vel = (x_des_rcm_.p - x_rcm_.p);
	    
	    x_err_vel_.vel = (x_err_.vel - x_err_last_.vel)/period.seconds();
	    x_rcm_err_vel_.vel = (x_rcm_err_.vel - x_rcm_err_last_.vel)/period.seconds();


	    x_err_last_.vel = x_err_.vel;
	    x_rcm_err_last_.vel = x_rcm_err_.vel;	 
	    //ROS_INFO("rcm error: %f, %f, %f",x_rcm_err_.vel(0),x_rcm_err_.vel(1),x_rcm_err_.vel(2));

            //ROS_INFO("Sampling period: %lf", period.toSec());


	    Eigen::MatrixXd mat(7,1);
            for (int i = 0; i < J_tooltip_pinv_.rows(); i++)
            {

		mat(i,0) = 0;
                for (int k = 0; k < 3; k++)
                    {
			mat(i,0)+= J_end_pinv_(i,k)*(Kp2*x_rcm_err_(k)+Kd2*x_rcm_err_vel_(k)) ;
			
			}
		
            }
		mat = N_*mat;
		for (int i=0;i<J_tooltip_pinv_.rows();i++)
		{ //std::cout << mat(i,0) << " ";
                	joint_des_states_.qdot(i) = mat(i,0);
		//ROS_INFO("joint RCM value for %i joint: %f", i, mat(i,0));
			for (int k = 0; k < 3; k++)
				joint_des_states_.qdot(i) += J_tooltip_pinv_(i,k)*(Kp1 * x_err_(k) + Kd1 * x_err_vel_(k)) ;
		//ROS_INFO("joint value for %i joint: %f", i,joint_des_states_.qdot(i));
		}//std::cout<<std::endl;

            // integrating q_dot -> getting q (Euler method)
            for (int i = 0; i < joint_handles_.size(); i++)
                //joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);
                joint_des_states_.q(i) += period.seconds()*joint_des_states_.qdot(i);

            // joint limits saturation
            for (int i =0;  i < joint_handles_.size(); i++)
            {
                if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
            }

	    
            //ROS_INFO("Position error: %f, %f, %f", x_err_.vel[0],x_err_.vel[1],x_err_.vel[2]);
	    RCLCPP_INFO(nh_.node()->get_logger(), "RCM error: %f, %f, %f", x_rcm_err_.vel[0]*1000,x_rcm_err_.vel[1]*1000,x_rcm_err_.vel[2]*1000);
	    //errorfile << x_err_.vel[0] <<";"<< x_err_.vel[1]<<";"<<x_err_.vel[2]<< x_rcm_err_.vel[0] <<";"<< x_rcm_err_.vel[1]<<";"<<x_rcm_err_.vel[2]<<std::endl;
	    //xfile << x_.p[0]<<";"<<x_.p[1]<<";"<<x_.p[2]<<";"<< x_des_.p[0]<<";"<<x_des_.p[1]<<";"<<x_des_.p[2]<<";"<<std::endl;
	    //rcmfile << x_rcm_.p[0]<<";"<<x_rcm_.p[1]<<";"<<x_rcm_.p[2]<<";"<< x_des_rcm_.p[0]<<";"<<x_des_rcm_.p[1]<<";"<<x_des_rcm_.p[2]<<";"<<std::endl;
            if (Equal(x_.p, x_des_.p, 0.0005) && Equal(x_rcm_.p, x_des_rcm_.p, 0.0005))
            {
                x_.M.GetRPY(end_orientation(0),end_orientation(1),end_orientation(2));
                //ROS_INFO("On target");
                //ROS_INFO("END position: %lf,%lf,%lf", x_.p[0],x_.p[1],x_.p[2]);
                //ROS_INFO("END orientation: %lf,%lf,%lf", end_orientation(0),end_orientation(1),end_orientation(2));               
                cmd_flag_ = 0;
            }
        }

        // set controls for joints
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand(joint_des_states_.q(i));
        }
    }








    void HierarchicalController::command(const lwr_controllers::msg::PoseRPY::ConstSharedPtr &msg)
    {
        KDL::Frame frame_des_;

        switch(msg->id)
        {
            case 0:
            frame_des_ = KDL::Frame(
                    KDL::Rotation::RPY(msg->orientation.roll,
                                      msg->orientation.pitch,
                                      msg->orientation.yaw),
                    KDL::Vector(msg->position.x,
                                msg->position.y,
                                msg->position.z));
            break;

            case 1: // position only
            frame_des_ = KDL::Frame(
                KDL::Vector(msg->position.x,
                            msg->position.y,
                            msg->position.z));
            break;

            case 2: // orientation only
            frame_des_ = KDL::Frame(
                KDL::Rotation::RPY(msg->orientation.roll,
                                   msg->orientation.pitch,
                                   msg->orientation.yaw));
            break;

            default:
            RCLCPP_INFO(nh_.node()->get_logger(), "Wrong message ID");
            return;
        }

        x_des_ = frame_des_;

        cmd_flag_ = 1;
    }
}

PLUGINLIB_EXPORT_CLASS(kuka_controllers::HierarchicalController, controller_interface::ControllerInterface)
