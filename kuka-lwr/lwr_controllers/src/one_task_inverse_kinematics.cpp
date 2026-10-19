#include <iostream>
#include <lwr_controllers/one_task_inverse_kinematics.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>

#include <math.h>
#include <algorithm>

int des_flag=1;

namespace
{
    // Safety limits for the damped IK + joint-increment guard, per
    // safe_inverse_kinematics_failure_analysis.md. These are conservative
    // defaults, not physical robot limits pulled from the URDF.
    const double kLambdaMin = 0.01;          // damping floor when well-conditioned
    const double kLambdaMax = 0.3;           // damping ceiling near singularity
    const double kSigmaThreshold = 0.1;      // sigma_min below which damping ramps up
    const double kSigmaHardStop = 0.01;      // sigma_min below which motion is held
    const double kMaxJointVel = 0.5;         // rad/s per-joint velocity cap
    const double kMaxJointAcc = 2.0;         // rad/s^2 per-joint acceleration cap
    const double kMaxDeltaQPerCycle = 0.02;  // rad, absolute per-cycle increment cap
    const double kSoftLimitMargin = 0.0873;  // rad (~5 deg) soft joint-limit margin

    // Persistent qdot state for acceleration limiting across update() cycles.
    // Cannot live in the controller class without touching the header, so it
    // is kept here at file scope and reset from starting().
    Eigen::VectorXd g_qdot_prev;
    bool g_qdot_prev_valid = false;
}

namespace lwr_controllers
{
    OneTaskInverseKinematics::OneTaskInverseKinematics() {}
    OneTaskInverseKinematics::~OneTaskInverseKinematics() {}
 
    bool OneTaskInverseKinematics::configure(lwr_controllers::NodeParameters &n)
    {
        if( !(KinematicChainControllerBase<lwr_controllers::PositionInterface>::configure(n)) )
        {
            RCLCPP_ERROR(nh_.node()->get_logger(), "Couldn't initilize OneTaskInverseKinematics controller.");
            return false;
        }

        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

        q_cmd_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());

        if (!n.getParam("task_space_gain", task_space_gain_)) task_space_gain_ = 1.0;
        if (!std::isfinite(task_space_gain_) || task_space_gain_ < 0.1 || task_space_gain_ > 3.0)
        {
            RCLCPP_ERROR(nh_.node()->get_logger(), "task_space_gain must be finite and in [0.1, 3.0], got %.3f", task_space_gain_);
            return false;
        }
        RCLCPP_INFO(nh_.node()->get_logger(), "OneTaskInverseKinematics: task_space_gain=%.3f", task_space_gain_);

        // TODO(ros2-migration): ROS 1 read the joint handles here (controller load). The
        // ros2_control state interfaces are loaned only at activation, so these reads return
        // 0 here; starting() re-syncs the desired state with the measured one (as in ROS 1).
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
        x_des_ = x_;
        float tool_length;
        // tool_length=0.22; //0.22 is tool tip length
        tool_length=0.252;
        // tool_length=0.275;//0.330
        
        float init_tip[]={};
        init_tip[0]=x_des_.M(0,2)*tool_length+x_des_.p[0];
        init_tip[1]=x_des_.M(1,2)*tool_length+x_des_.p[1];
        init_tip[2]=x_des_.M(2,2)*tool_length+x_des_.p[2];
        RCLCPP_INFO(nh_.node()->get_logger(), "Init position: %lf,%lf,%lf", x_des_.p[0],x_des_.p[1],x_des_.p[2]);
        RCLCPP_INFO(nh_.node()->get_logger(), "Init X_axis: %lf,%lf,%lf", x_des_.M(0,0),x_des_.M(1,0),x_des_.M(2,0));
        RCLCPP_INFO(nh_.node()->get_logger(), "Init Y_axis: %lf,%lf,%lf", x_des_.M(0,1),x_des_.M(1,1),x_des_.M(2,1));
        RCLCPP_INFO(nh_.node()->get_logger(), "Init Z_axis: %lf,%lf,%lf", x_des_.M(0,2),x_des_.M(1,2),x_des_.M(2,2));

        cmd_flag_ = 0;

        
        sub_command_ = nh_.subscribe("command", 1, &OneTaskInverseKinematics::command, this);

        return true;
    }

    void OneTaskInverseKinematics::starting(const rclcpp::Time& time)
    {
        // Re-sync the desired state with the robot's actual pose every time
        // this controller is (re)started. Without this, joint_des_states_/x_des_
        // stay frozen at whatever pose was captured once in init() (controller
        // load time), so switching into this controller later would command the
        // arm back to that stale pose instead of holding its current position.
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
        x_des_ = x_;

        cmd_flag_ = 0;
        g_qdot_prev_valid = false;
    }

    void OneTaskInverseKinematics::update_command(const rclcpp::Time& time, const rclcpp::Duration& period)
    {

        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }

        if (cmd_flag_)
        {
            // computing Jacobian
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

            // computing J_pinv_ with adaptive damping: damping stays low
            // when the Jacobian is well-conditioned and increases as the
            // smallest singular value shrinks, instead of the legacy
            // undamped/fixed-lambda pseudoinverse. Doing the SVD here (not
            // via utils/pseudo_inverse()) also gives us sigma_min for the
            // singularity hard stop below.
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_.data, Eigen::ComputeThinU | Eigen::ComputeThinV);
            const Eigen::VectorXd &sigma = svd.singularValues();
            double sigma_min = sigma(sigma.size() - 1);

            double lambda = kLambdaMin;
            if (sigma_min < kSigmaThreshold)
            {
                double ratio = sigma_min / kSigmaThreshold;
                lambda = kLambdaMin + (kLambdaMax - kLambdaMin) * (1.0 - ratio) * (1.0 - ratio);
            }

            Eigen::MatrixXd S = Eigen::MatrixXd::Zero(sigma.size(), sigma.size());
            for (int i = 0; i < sigma.size(); i++)
                S(i,i) = sigma(i) / (sigma(i)*sigma(i) + lambda*lambda);
            J_pinv_ = svd.matrixV() * S * svd.matrixU().transpose();

            // hard stop: do not attempt to recover from a near-singular
            // configuration with ever-heavier damping, just hold position.
            bool singularity_hold = (sigma_min < kSigmaHardStop);
            if (singularity_hold)
                RCLCPP_WARN_THROTTLE(nh_.node()->get_logger(), *nh_.node()->get_clock(), 1000, "OneTaskInverseKinematics: near singularity (sigma_min=%f), holding position", sigma_min);

            // computing forward kinematics
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);


            
            KDL::Frame x_des_new;
            x_des_new=x_des_;
            float tool_length;
            // tool_length=0.22;
            // tool_length=0.0;
            // tool_length=0.275; // Real length for Tool3
            tool_length=0.0; 
          
            float des_tip[]={};
            des_tip[0]=x_des_.M(0,2)*tool_length;
            des_tip[1]=x_des_.M(1,2)*tool_length;
            des_tip[2]=x_des_.M(2,2)*tool_length;
            x_des_new.p[0]=x_des_.p[0]-des_tip[0];
            x_des_new.p[1]=x_des_.p[1]-des_tip[1];
            x_des_new.p[2]=x_des_.p[2]-des_tip[2];
            
            x_des_new.M.GetRPY(goal_orientation(0),goal_orientation(1),goal_orientation(2));
            
            //RCLCPP_INFO(nh_.node()->get_logger(), "Goal position: %lf,%lf,%lf,%lf", x_des_new.p[0]+des_tip[0],x_des_new.p[1]+des_tip[1],x_des_new.p[2]+des_tip[2],period.seconds());
            //RCLCPP_INFO(nh_.node()->get_logger(), "Goal Orientation: %lf,%lf,%lf", goal_orientation[0],goal_orientation[1],goal_orientation[2]);
            
            // end-effector position error
            x_err_.vel = x_des_new.p - x_.p;

            // getting quaternion from rotation matrix
            x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
            x_des_new.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

            skew_symmetric(quat_des_.v, skew_);

            for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;
                for (int k = 0; k < skew_.cols(); k++)
                    v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
            }

            // end-effector orientation error
            x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;

            // computing q_dot
            Eigen::VectorXd qdot = Eigen::VectorXd::Zero(J_pinv_.rows());
            if (!singularity_hold)
            {
                for (int i = 0; i < J_pinv_.rows(); i++)
                    for (int k = 0; k < J_pinv_.cols(); k++)
                        qdot(i) += task_space_gain_ * J_pinv_(i,k)*x_err_(k);

                // per-joint velocity limit
                for (int i = 0; i < qdot.size(); i++)
                {
                    if (qdot(i) > kMaxJointVel) qdot(i) = kMaxJointVel;
                    if (qdot(i) < -kMaxJointVel) qdot(i) = -kMaxJointVel;
                }

                // per-joint acceleration limit (relative to last commanded qdot)
                if (g_qdot_prev_valid)
                {
                    double max_dqdot = kMaxJointAcc * period.seconds();
                    for (int i = 0; i < qdot.size(); i++)
                    {
                        double lo = g_qdot_prev(i) - max_dqdot;
                        double hi = g_qdot_prev(i) + max_dqdot;
                        if (qdot(i) < lo) qdot(i) = lo;
                        if (qdot(i) > hi) qdot(i) = hi;
                    }
                }
            }

            // integrating q_dot -> getting q (Euler method, actual control period)
            Eigen::VectorXd delta_q = period.seconds() * qdot;

            // per-cycle joint-increment cap: if any joint would exceed it,
            // scale the whole increment down uniformly (reducing the
            // effective step size) instead of independently saturating
            // each joint, which would distort the commanded direction.
            double max_abs_delta = 0.0;
            for (int i = 0; i < delta_q.size(); i++)
                max_abs_delta = std::max(max_abs_delta, std::fabs(delta_q(i)));
            if (max_abs_delta > kMaxDeltaQPerCycle)
                delta_q *= (kMaxDeltaQPerCycle / max_abs_delta);

            // soft joint limits: ramp the increment down to zero as a joint
            // nears its limit while still moving toward it, instead of
            // moving at full speed and truncating only at the last instant.
            for (int i = 0; i < joint_handles_.size(); i++)
            {
                double next_q = joint_des_states_.q(i) + delta_q(i);
                if (delta_q(i) < 0.0)
                {
                    double dist_to_min = next_q - joint_limits_.min(i);
                    if (dist_to_min < kSoftLimitMargin)
                        delta_q(i) *= std::max(0.0, dist_to_min / kSoftLimitMargin);
                }
                else if (delta_q(i) > 0.0)
                {
                    double dist_to_max = joint_limits_.max(i) - next_q;
                    if (dist_to_max < kSoftLimitMargin)
                        delta_q(i) *= std::max(0.0, dist_to_max / kSoftLimitMargin);
                }
            }

            for (int i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.q(i) += delta_q(i);

            // joint limits saturation (final hard safety net)
            for (int i =0;  i < joint_handles_.size(); i++)
            {
                if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
            }

            // record the actually-applied velocity (post per-cycle-cap and
            // soft-limit scaling) for next cycle's acceleration limit and
            // for the qdot state reported on the joint interface.
            double dt = period.seconds();
            Eigen::VectorXd applied_qdot = (dt > 1e-9) ? (delta_q / dt) : qdot;
            for (int i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.qdot(i) = applied_qdot(i);
            g_qdot_prev = applied_qdot;
            g_qdot_prev_valid = true;
            x_.M.GetRPY(end_orientation(0),end_orientation(1),end_orientation(2));
                           
            if (Equal(x_, x_des_new, 0.00005)) // orignal 0.05, should be wrong
            {
                x_.M.GetRPY(end_orientation(0),end_orientation(1),end_orientation(2));
                RCLCPP_INFO(nh_.node()->get_logger(), "On target");
                RCLCPP_INFO(nh_.node()->get_logger(), "END position: %lf,%lf,%lf", x_.p[0]+x_.M(0,2)*tool_length,x_.p[1]+x_.M(1,2)*tool_length,x_.p[2]+x_.M(2,2)*tool_length);
                RCLCPP_INFO(nh_.node()->get_logger(), "END orientation: %lf,%lf,%lf", end_orientation(0),end_orientation(1),end_orientation(2));               
                RCLCPP_INFO(nh_.node()->get_logger(), "THIS IS THE ONE");
                cmd_flag_ = 0;
            }
        }

        // set controls for joints
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand(joint_des_states_.q(i));
        }
    }

    void OneTaskInverseKinematics::command(const lwr_controllers::msg::PoseRPY::ConstSharedPtr &msg)
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

PLUGINLIB_EXPORT_CLASS(lwr_controllers::OneTaskInverseKinematics, controller_interface::ControllerInterface)
