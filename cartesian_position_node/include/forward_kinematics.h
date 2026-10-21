

#include <urdf/model.h>
#include <rclcpp/rclcpp.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <utils/pseudo_inversion.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarrayvel.hpp>
#ifndef FORWARD_KINEMATICS
#define FORWARD_KINEMATICS

#include <iostream>


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

    J(3,0) = 0;
    J(3,1) = 0;
    J(3,2) = 0;
    J(3,3) = 0;
    J(3,4) = 0;
    J(3,5) = 0;
    J(3,6) = 0;

    J(4,0) = 0;
    J(4,1) = 0;
    J(4,2) = 0;
    J(4,3) = 0;
    J(4,4) = 0;
    J(4,5) = 0;
    J(4,6) = 0;

    J(5,0) = 0;
    J(5,1) = 0;
    J(5,2) = 0;
    J(5,3) = 0;
    J(5,4) = 0;
    J(5,5) = 0;
    J(5,6) = 0;

}


class ForwardKinematics{


public:

	ForwardKinematics(){}
	~ForwardKinematics(){}
	// ROS 2: the global parameters read in ROS 1 (/robot_description found by
	// ros::param::search, /lwr/root, /lwr/tip) are parameters of this node:
	// robot_description, lwr.root, lwr.tip
	static bool getParam(const rclcpp::Node::SharedPtr &n, const std::string &name, std::string &value)
	{
		if (!n->has_parameter(name))
			n->declare_parameter(name, rclcpp::ParameterValue{}, rcl_interfaces::msg::ParameterDescriptor().set__dynamic_typing(true));
		const auto parameter = n->get_parameter(name);
		if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) return false;
		value = parameter.as_string();
		return true;
	}

	bool init(rclcpp::Node::SharedPtr n)
	{
		nh_ = n;
		std::string robot_description, root_name, tip_name;

		robot_description = "robot_description";
		std::string robot_description_probe;
		if (!getParam(n, robot_description, robot_description_probe))
		{
			RCLCPP_ERROR_STREAM(nh_->get_logger(), "KinematicChainControllerBase: No robot description (URDF) found on parameter server ("<<n->get_namespace()<<"/robot_description)");
			return false;
		}

		if (!getParam(nh_, "lwr.root", root_name))
		{
			RCLCPP_ERROR_STREAM(nh_->get_logger(), "KinematicChainControllerBase: No root name found on parameter server ("<<n->get_namespace()<<"/root_name)");
			return false;
		}
		RCLCPP_INFO(nh_->get_logger(), "root %s",root_name.c_str()); 
		if (!getParam(nh_, "lwr.tip", tip_name))
		{
			RCLCPP_ERROR_STREAM(nh_->get_logger(), "KinematicChainControllerBase: No tip name found on parameter server ("<<n->get_namespace()<<"/tip_name)");
			return false;
		}	
		RCLCPP_INFO(nh_->get_logger(), "tip %s",tip_name.c_str());
		std::string xml_string;

        	if (getParam(n, robot_description.c_str(), xml_string))
           		;
       		else
        	{
            		RCLCPP_ERROR(nh_->get_logger(), "Parameter %s not set, shutting down node...", robot_description.c_str());
            		// TODO(ros2-migration): ROS 1 n.shutdown()
            		return false;
        	}

		if (xml_string.size() == 0)
		{
		    RCLCPP_ERROR(nh_->get_logger(), "Unable to load robot model from parameter %s",robot_description.c_str());
		    // TODO(ros2-migration): ROS 1 n.shutdown()
		    return false;
		}

		urdf::Model model;
		if (!model.initString(xml_string))
		{
		    RCLCPP_ERROR(nh_->get_logger(), "Failed to parse urdf file");
		    // TODO(ros2-migration): ROS 1 n.shutdown()
		    return false;
		}
		RCLCPP_INFO(nh_->get_logger(), "Successfully parsed urdf file");
		
		KDL::Tree kdl_tree_;
		if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
		{
		    RCLCPP_ERROR(nh_->get_logger(), "Failed to construct kdl tree");
		    // TODO(ros2-migration): ROS 1 n.shutdown()
		    return false;
		}

		// Populate the KDL chain
		if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
		{
		    RCLCPP_ERROR_STREAM(nh_->get_logger(), "KDL chain from tree: ");
		    RCLCPP_ERROR_STREAM(nh_->get_logger(), "  "<<root_name<<" --> "<<tip_name);
		    RCLCPP_ERROR_STREAM(nh_->get_logger(), "  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
		    RCLCPP_ERROR_STREAM(nh_->get_logger(), "  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
		    RCLCPP_ERROR_STREAM(nh_->get_logger(), "  The segments are:");

		    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
		    KDL::SegmentMap::iterator it;

		    for( it=segment_map.begin(); it != segment_map.end(); it++ )
		      RCLCPP_ERROR_STREAM(nh_->get_logger(),  "    "<<(*it).first);

		    return false;
		}
        	RCLCPP_INFO(nh_->get_logger(), "Number of segments: %d", kdl_chain_.getNrOfSegments());
        	RCLCPP_INFO(nh_->get_logger(), "Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
		jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
		fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
		J_.resize(kdl_chain_.getNrOfJoints());
		return true;

	}



	void compute(KDL::JntArray& joint_states, KDL::Frame& x){
            // computing forward kinematics
            fk_pos_solver_->JntToCart(joint_states, x);


	}
	void computeVel(KDL::JntArray& joint_states, KDL::JntArray& joint_states_velocities, std::vector<double>& x_dot,float d1,float d2,float d3){
		KDL::JntArrayVel j_vel(joint_states, joint_states_velocities);

		KDL::FrameVel frame_vel;

		fk_vel_solver_->JntToCart(j_vel,frame_vel);

		//KDL::Twist vel = frame_vel.GetTwist();

		KDL::Rotation R = frame_vel.GetFrame().M;
		KDL::Rotation Rt = KDL::Rotation(R(0,0),R(1,0),R(2,0),R(0,1),R(1,1),R(2,1),R(0,2),R(1,2),R(2,2));

		Jacobian(joint_states,J_,d1,d2,d3);

		for (int i=0;i<3;i++){
    			x_dot.at(i) = 0;

			for (int k = 0; k < 7; k++)
				x_dot.at(i) += J_(i,k)*joint_states_velocities(k);
		}


	}


private:
	KDL::Chain kdl_chain_;
	rclcpp::Node::SharedPtr nh_;
	KDL::Jacobian J_;	//Jacobian
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Jacobian Solver
	Eigen::MatrixXd J_pinv_;
	boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; // Forward kinematics solver	
	boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;


};


#endif
