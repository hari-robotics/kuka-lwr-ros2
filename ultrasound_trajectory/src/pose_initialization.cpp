#include "ultrasound_trajectory/ros2_support.hpp"
#include <functional>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
//	kuka-lwr Controllers Msgs
#include <lwr_controllers/msg/arm_state.hpp>
#include <lwr_controllers/msg/pose_rpy.hpp>
#include <lwr_controllers/msg/cartesian_impedance_point.hpp>
//	Header Files
#include "ultrasound_trajectory/smooth_traj.h"
//	Eigen lib
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Geometry>

using namespace std;

class UltrasoundTrajNode : public rclcpp::Node {
    private:
	rclcpp::Subscription<lwr_controllers::msg::PoseRPY>::SharedPtr sub_pose;
    lwr_controllers::msg::PoseRPY::ConstSharedPtr latest_pose_;
    rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
    rclcpp::TimerBase::SharedPtr trajectory_timer_;
    double feedback_timeout_{0.5};
 		// Position, Orientation Subscriber

    rclcpp::Publisher<lwr_controllers::msg::CartesianImpedancePoint>::SharedPtr pub_command; 	// Impedance Controller Command

    lwr_controllers::msg::CartesianImpedancePoint command_msg;
    string frame_id = "0";

    double ee_position_x{0}; double ee_position_y{0}; double ee_position_z{0};	//	EE POSITION (retrieved continuously)
	double roll{0}; double pitch{0}; double yaw{0};	//	EE PRY ORIENTATION
	double q_x{0}; double q_y{0}; double q_z{0}; double q_w{0};	//	EE QUAT ORIENTATION
    int flag_init = 1;
    //TODO: potrei collegarmi al topic di cartesian pose e cambiare solo il valore dell`asse che voglio cambiare (piu efficiente) 
    double desired_position_x = -0.5840; // -0.42 valore per robot verticale
    double desired_position_y = 0.3454; // 0.53
    double desired_position_z =  0.1603; /// 0.48
    double desired_roll = 1.5708; // -3.13
    double desired_pitch = 0.0; // -0.03
    double desired_yaw = 3.1416; // 1.84
    double desired_q_x = 0.0;
    double desired_q_y = 0.0;
    double desired_q_z = 0.0;
    double desired_q_w = 0.0;

    int SubSteps = 100;
    int i_home = 0;
    int T = 10;
    int i = 0;

    double xt{0}; double yt{0}; double zt{0};
    double xi{0}; double yi{0}; double zi{0};
    double q_xt{0}; double q_yt{0}; double q_zt{0}; double q_wt{0};
    double q_xi{0}; double q_yi{0}; double q_zi{0}; double q_wi{0};

    bool printed = false;

    public:
    UltrasoundTrajNode() : rclcpp::Node("pose_initialization") {
        feedback_timeout_ = ultrasound_trajectory::startup_parameter(*this, "feedback_timeout", feedback_timeout_);
        trajectory_timer_ = rclcpp::create_timer(this, get_clock(),
            rclcpp::Duration::from_seconds(0.01), [this]() {
                if (latest_pose_ && (now() - last_pose_time_).seconds() <= feedback_timeout_) {
                    run_trajectory(latest_pose_);
                }
            });

        sub_pose = create_subscription<lwr_controllers::msg::PoseRPY>("/cartesian_position/pose/end_effector", rclcpp::SensorDataQoS(), std::bind(&UltrasoundTrajNode::callback_pose, this, std::placeholders::_1));
        pub_command = create_publisher<lwr_controllers::msg::CartesianImpedancePoint>("/lwr/cartesian_impedance_controller/command", 10);
    }

    void callback_pose(const lwr_controllers::msg::PoseRPY::ConstSharedPtr& msg) {
        if (!ultrasound_trajectory::finite_pose(*msg)) {return;}
        latest_pose_ = msg;
        last_pose_time_ = now();
    }

    void run_trajectory(const lwr_controllers::msg::PoseRPY::ConstSharedPtr& pose_msg) {
        ee_position_x = pose_msg->position.x;
        ee_position_y = pose_msg->position.y;
        ee_position_z = pose_msg->position.z;
        roll = pose_msg->orientation.roll;
        pitch = pose_msg->orientation.pitch;
        yaw = pose_msg->orientation.yaw;
       
       // Convert RPY to quaternion XYZW
		q_x = (sin(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5)) - (cos(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5));
		q_y = (cos(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5)) + (sin(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5));
		q_z = (cos(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5)) - (sin(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5));		
		q_w = (cos(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5)) + (sin(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5));
        
        // Set the desired pose
         // step di 0.01 ad ogni passo
        double t = 0.0; // parametro di interpolazione

        if(flag_init == 1){
            // Position
			command_msg.header.frame_id = frame_id;		
			command_msg.x_fri.position.x = ee_position_x;
			command_msg.x_fri.position.y = ee_position_y;
			command_msg.x_fri.position.z = ee_position_z;
			// Orientation
			command_msg.x_fri.orientation.x = q_x; command_msg.x_fri.orientation.y = q_y; command_msg.x_fri.orientation.z = q_z; command_msg.x_fri.orientation.w = q_w;
            // Stiffness
            command_msg.k_fri.x = 1500.0; command_msg.k_fri.y = 1500.0; command_msg.k_fri.z = 1500; 
            command_msg.k_fri.rx = 250.0; command_msg.k_fri.ry = 250.0; command_msg.k_fri.rz = 250.0;
            // Damping
            command_msg.d_fri.x = 0.8; command_msg.d_fri.y = 0.8; command_msg.d_fri.z = 0.8;
            command_msg.d_fri.rx = 0.8; command_msg.d_fri.ry = 0.8; command_msg.d_fri.rz = 0.8;    
            // Force
            command_msg.f_fri.force.x = 0.0; 
            command_msg.f_fri.force.y = 0.0;
            command_msg.f_fri.force.z = 0.0; 
            // Torque		
            command_msg.f_fri.torque.x = 0.0;
            command_msg.f_fri.torque.y = 0.0;
            command_msg.f_fri.torque.z = 0.0;
            // Save initial position
            xi = ee_position_x;
            yi = ee_position_y;
            zi = ee_position_z;
            // Save initial orientation
            q_xi = q_x;
            q_yi = q_y;
            q_zi = q_z;
            q_wi = q_w;
            // Define desired orientation
            desired_q_x = (sin(desired_roll*0.5) * cos(desired_pitch*0.5) * cos(desired_yaw*0.5)) - (cos(desired_roll*0.5) * sin(desired_pitch*0.5) * sin(desired_yaw*0.5));
            desired_q_y = (cos(desired_roll*0.5) * sin(desired_pitch*0.5) * cos(desired_yaw*0.5)) + (sin(desired_roll*0.5) * cos(desired_pitch*0.5) * sin(desired_yaw*0.5));
            desired_q_z = (cos(desired_roll*0.5) * cos(desired_pitch*0.5) * sin(desired_yaw*0.5)) - (sin(desired_roll*0.5) * sin(desired_pitch*0.5) * cos(desired_yaw*0.5));		
            desired_q_w = (cos(desired_roll*0.5) * cos(desired_pitch*0.5) * cos(desired_yaw*0.5)) + (sin(desired_roll*0.5) * sin(desired_pitch*0.5) * sin(desired_yaw*0.5));
            
            flag_init = 0;
        }    
        
        
        
        if(i <= T * SubSteps){
            RCLCPP_INFO(get_logger(), "ee_position_x: %f", ee_position_x);
            RCLCPP_INFO(get_logger(), "ee_position_y: %f", ee_position_y);
            RCLCPP_INFO(get_logger(), "ee_position_z: %f", ee_position_z);

            
            // Alternativa con smooth traj
            zt = SmoothTraj(zi, desired_position_z, T, SubSteps, i_home);
            xt = SmoothTraj(xi, desired_position_x, T, SubSteps, i_home);
            yt = SmoothTraj(yi, desired_position_y, T, SubSteps, i_home);

            command_msg.x_fri.position.x = xt;
            command_msg.x_fri.position.y = yt;
            command_msg.x_fri.position.z = zt;     

            /*
            Eigen::Quaterniond current_orientation(q_wi, q_xi, q_yi, q_zi);
            Eigen::Quaterniond desired_orientation(desired_q_w, desired_q_x, desired_q_y, desired_q_z);
            Eigen::Quaterniond interpolated_orientation = current_orientation.slerp(t, desired_orientation);         
            
            // Update the end effector's orientation
            q_xt = interpolated_orientation.x();
            q_yt = interpolated_orientation.y();
            q_zt = interpolated_orientation.z();
            q_wt = interpolated_orientation.w();

            command_msg.x_fri.orientation.x = q_xt;
            command_msg.x_fri.orientation.y = q_yt;
            command_msg.x_fri.orientation.z = q_zt;
            command_msg.x_fri.orientation.w = q_wt;
            
            RCLCPP_INFO(get_logger(), "Update x: %f", ee_position_x);
            RCLCPP_INFO(get_logger(), "Update y: %f", ee_position_y);
            RCLCPP_INFO(get_logger(), "Update z: %f", ee_position_z);
            */

            q_zt = SmoothTraj(q_zi, desired_q_z, T, SubSteps, i_home);
            q_xt = SmoothTraj(q_xi, desired_q_x, T, SubSteps, i_home);
            q_yt = SmoothTraj(q_yi, desired_q_y, T, SubSteps, i_home);
            q_wt = SmoothTraj(q_wi, desired_q_w, T, SubSteps, i_home);

            command_msg.x_fri.orientation.x = q_xt;
            command_msg.x_fri.orientation.y = q_yt;
            command_msg.x_fri.orientation.z = q_zt;
            command_msg.x_fri.orientation.w = q_wt;
            
            pub_command->publish(command_msg);
            
            //
            t += 0.01;
            i++;
            i_home++;
            
        }    

        if (i > T * SubSteps && i <= 2 * T * SubSteps)
		{
			command_msg.x_fri.position.x = xt;
			command_msg.x_fri.position.y = yt;
			command_msg.x_fri.position.z = zt;

            command_msg.x_fri.orientation.x = q_xt;
            command_msg.x_fri.orientation.y = q_yt;
            command_msg.x_fri.orientation.z = q_zt;
            command_msg.x_fri.orientation.w = q_wt;

			// command_msg.k_fri.z = K_z;
			pub_command->publish(command_msg);

			i++;
			//	RCLCPP_INFO(get_logger(), "i: %d, x_cmd: %f, y_cmd: %f, z_cmd: %f", i, xi, yi, zt);
			RCLCPP_INFO(get_logger(), "Reached the final position.");
			
			
		}

		if (i > 2*T*SubSteps){

		rclcpp::shutdown();
		}
        
        /*while (sqrt(pow(desired_position_x - ee_position_x, 2) + pow(desired_position_y - ee_position_y, 2) + pow(desired_position_z - ee_position_z, 2)) > 0.001) { 
            // finchè la distanza tra la posizione attuale e quella desiderata è maggiore di 0.001
            RCLCPP_INFO(get_logger(), "ee_position_x: %f", ee_position_x);
            RCLCPP_INFO(get_logger(), "ee_position_y: %f", ee_position_y);
            RCLCPP_INFO(get_logger(), "ee_position_z: %f", ee_position_z);

            
            // Calcolo la direzione del passo
            double direction_x = (desired_position_x - ee_position_x); // direzione del passo in x
            double direction_y = (desired_position_y - ee_position_y); // direzione del passo in y
            double direction_z = (desired_position_z - ee_position_z); // direzione del passo in z
            double length = sqrt(pow(direction_x, 2) + pow(direction_y, 2) + pow(direction_z, 2));  
            direction_x /= length; 
            direction_y /= length;
            direction_z /= length;

            // Muovo end effector di un passo nella direzione corretta
            ee_position_x += step * direction_x;                
            ee_position_y += step * direction_y;
            ee_position_z += step * direction_z;

            command_msg.x_fri.position.x = ee_position_x;
            command_msg.x_fri.position.y = ee_position_y;
            command_msg.x_fri.position.z = ee_position_z;                
            
            RCLCPP_INFO(get_logger(), "Update x: %f", ee_position_x);
            RCLCPP_INFO(get_logger(), "Update y: %f", ee_position_y);
            RCLCPP_INFO(get_logger(), "Update z: %f", ee_position_z);
            

            / *
            // Alternativa con smooth traj
            zt = SmoothTraj(ee_position_z, desired_position_z, T, SubSteps, i_home);
            xt = SmoothTraj(ee_position_x, desired_position_x, T, SubSteps, i_home);
            yt = SmoothTraj(ee_position_y, desired_position_y, T, SubSteps, i_home);

            command_msg.x_fri.position.x = xt;
            command_msg.x_fri.position.y = yt;
            command_msg.x_fri.position.z = zt;

            RCLCPP_INFO(get_logger(), "Update x: %f", xt);
            RCLCPP_INFO(get_logger(), "Update y: %f", yt);
            RCLCPP_INFO(get_logger(), "Update z: %f", zt);

            
            // Publishing
            pub_command->publish(command_msg);

            / *
            // Interpolate orientation --> ad ogni iterazione calcola un quaternion intermedio tra corrente e desiderato, con parametro di interpolazione t
            Eigen::Quaterniond current_orientation(q_w, q_x, q_y, q_z);
            Eigen::Quaterniond desired_orientation(desired_q_w, desired_q_x, desired_q_y, desired_q_z);
            Eigen::Quaterniond interpolated_orientation = current_orientation.slerp(t, desired_orientation);
            RCLCPP_INFO(get_logger(), "Current q_x: %f", q_x);
            RCLCPP_INFO(get_logger(), "Current q_y: %f", q_y);
            RCLCPP_INFO(get_logger(), "Current q_z: %f", q_z);
            RCLCPP_INFO(get_logger(), "Current q_w: %f", q_w);

            if(!printed){
                RCLCPP_INFO(get_logger(), "Desired q_x: %f", desired_q_x);
                RCLCPP_INFO(get_logger(), "Desired q_y: %f", desired_q_y);
                RCLCPP_INFO(get_logger(), "Desired q_z: %f", desired_q_z);
                RCLCPP_INFO(get_logger(), "Desired q_w: %f", desired_q_w);
            }
            printed = true;

            // Update the end effector's orientation
            q_x = interpolated_orientation.x();
            q_y = interpolated_orientation.y();
            q_z = interpolated_orientation.z();
            q_w = interpolated_orientation.w();

            command_msg.x_fri.orientation.x = q_x;
            command_msg.x_fri.orientation.y = q_y;
            command_msg.x_fri.orientation.z = q_z;
            command_msg.x_fri.orientation.w = q_w;

            RCLCPP_INFO(get_logger(), "Update q_x: %f", q_x);
            RCLCPP_INFO(get_logger(), "Update q_y: %f", q_y);
            RCLCPP_INFO(get_logger(), "Update q_z: %f", q_z);
            RCLCPP_INFO(get_logger(), "Update q_w: %f", q_w);
                
            // Publishing
            pub_command->publish(command_msg);

            
            
            t += 0.01;
            i_home++;
        }*/
        
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<UltrasoundTrajNode>());
    } catch (const std::exception & error) {
        RCLCPP_ERROR(rclcpp::get_logger("pose_initialization"), "%s", error.what());
        result = 1;
    }
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return result;
}
