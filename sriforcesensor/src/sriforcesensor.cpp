#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
#include <iostream>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <signal.h>
#include "TCPClient.h"
#include <eigen3/Eigen/Dense>
#include <math.h>
#include "lwr_controllers/msg/pose_rpy.hpp"
# define M_PI           3.14159265358979323846  /* pi */

using namespace Eigen;
using namespace std;
TCPClient tcp;
#define M812X_CHN_NUMBER	6
MatrixXd m_dResultChValue=MatrixXd::Zero(1,M812X_CHN_NUMBER);   //engineering output of each channel
MatrixXd m_dDecouplingValue=MatrixXd::Zero(1,M812X_CHN_NUMBER); //final output
MatrixXd m_nADCounts=MatrixXd::Zero(1,M812X_CHN_NUMBER);        //ad output
MatrixXd m_dAmpZero=MatrixXd::Zero(1,M812X_CHN_NUMBER);         //read
MatrixXd m_dChnGain=MatrixXd::Zero(1,M812X_CHN_NUMBER);         //read
MatrixXd m_dChnEx=MatrixXd::Zero(1,M812X_CHN_NUMBER);           //read
MatrixXd m_dDecouplingCoefficient(6, 6);
MatrixXd m_dSens=MatrixXd::Zero(1,M812X_CHN_NUMBER); ////

void sig_exit(int s)
{
  tcp.exit();
  exit(0);
}


bool ConfigSystem(void)
{

  //string rec = tcp.receive();
  tcp.Send("AT+SGDM=(A01,A02,A03,A04,A05,A06);C;1;(WMA:1)\r\n");
  string rec = tcp.read();
  if( rec != "" )
  {
    std::cout << "Server Response:" << rec << endl;
  } else
  {
    std::cout << "Server Not Response:"  << endl;
  }

  tcp.Send("AT+AMPZ=?\r\n");
  if(tcp.GetChParameter(m_dAmpZero))
  {
    std::cout << "AT+AMPZ=: " << m_dAmpZero << endl;
  }else{
    std::cout << "Server no Response:" << endl;
  }

  tcp.Send("AT+CHNAPG=?\r\n");
  if(tcp.GetChParameter(m_dChnGain))
  {
    std::cout << "AT+CHNAPG=: " << m_dChnGain << endl;
  }else{
    std::cout << "Server no Response:" << endl;
  }
  tcp.Send("AT+GOD\r\n");
  if(tcp.GetADCounts(m_nADCounts))
  {
    std::cout << "AD value:" << m_nADCounts << endl;
  }else{
    std::cout << "Server no Response:" << endl;
  }
  tcp.Send("AT+SENS=?\r\n");
  if(tcp.GetChParameter(m_dSens))
  {
    std::cout <<"Sensitivity value:" << m_dSens << endl;
  }
  else{
    std::cout << "Server no Response:" << endl;
  }
  for (unsigned  int i = 0; i < 6; ++i) {
    m_dResultChValue(i) = 1000*( (m_nADCounts(i) - m_dAmpZero(i)) / (double)65535*(double)5 ) / m_dChnGain(i);
  }
  std::cout << "result value is : "  <<m_dResultChValue << std::endl;

  m_dDecouplingCoefficient<<0.00012,-0.02812,-0.00028,0.02814,-0.00010,0.00008,
			0.00006,-0.01626,0.00012,-0.01627,0.00003,0.03236,
			-0.04569,0.00007,-0.04618,-0.00017,-0.04545,-0.00008,
			-0.00131,0.00001,0.00132,0.00001,0.00000,-0.00001,
			-0.00077,-0.00001,-0.00078,0.00000,0.00147,0.00000,
			0.00000,0.00104,-0.00001,0.00103,0.00000,0.00105;

  std::cout << "Decoupling matrix is set as: " <<endl <<m_dDecouplingCoefficient << std::endl;
  m_dDecouplingValue=m_dResultChValue*m_dDecouplingCoefficient.transpose();
  std::cout << "force (with offset) is: " <<endl <<m_dDecouplingValue << std::endl;
  return true;
}

/*0.000130,0.051905,0.000135,-0.052493,0.000118,0.000096,
                            0.000027,0.030204,0.000105,0.030377,0.000030,-0.060715,
                           0.047253,-0.000245,0.048106,-0.001334,0.048817,-0.000172,
                           0.001345,-0.000001,-0.001430,0.000056,-0.000031,-0.000024,
                           0.000785,-0.000040,0.000793,0.000016,-0.001608,0.000009,
                          -0.000004,-0.001925,0.000002,-0.001920,0.000002,-0.001960;
*/



int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>("talker");

  signal(SIGINT, sig_exit);
  if(tcp.setup("192.168.0.108",4008)==true)
  {
    cout<<"Force sensor has been connected!"<<endl;
  }else{
    cout<<"Force sensor connection failed!!!!!!!!!!!!!!!!!"<<endl;
  }
  //initialize the setting of the force sensor
  ConfigSystem();
  //get real time force sensor data
  tcp.Send("AT+GSD\r\n");

  geometry_msgs::msg::Twist Forcevalue;
  geometry_msgs::msg::TwistStamped Forcevalue_tmp;
  geometry_msgs::msg::TwistStamped Forcevalue_filtered;
  
  auto chatter_pub = n->create_publisher<geometry_msgs::msg::Twist>("Force", 1000);
  rclcpp::Rate loop_rate(1000);
 
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  //filtering param//
  double alpha = 0.05;
  //calibration param//
  float theta = -20 * (M_PI / 180); 
  Eigen::Matrix3f RotZ;
  Eigen::Vector3f Forcevalue_rot_force;
  Eigen::Vector3f Forcevalue_rot_torque;
  RotZ << cos(theta),-sin(theta),0, sin(theta),cos(theta),0,0,0,1;


  while (rclcpp::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    tcp.readrecieveBuffer(m_nADCounts);
    for (unsigned  int i = 0; i < 6; ++i) {
      m_dResultChValue(i) = 1000*( (m_nADCounts(i) - m_dAmpZero(i)) / (double)65535*(double)5 ) / m_dChnGain(i);
    }
    m_dDecouplingValue=m_dResultChValue*m_dDecouplingCoefficient.transpose();

    Forcevalue.linear.x=m_dDecouplingValue(0,0)-0.85; //REMOVE OFFSET
    Forcevalue.linear.y=m_dDecouplingValue(0,1)-1.00;
    Forcevalue.linear.z=m_dDecouplingValue(0,2)-1.25;
    Forcevalue.angular.x=m_dDecouplingValue(0,3);
    Forcevalue.angular.y=m_dDecouplingValue(0,4);
    Forcevalue.angular.z=m_dDecouplingValue(0,5);
    
    // FILTER WITH EXPONENTIAL SMOOTHING FILTER//
    Forcevalue_filtered.twist.linear.x= exponentialSmoothing(Forcevalue.linear.x,Forcevalue_tmp.twist.linear.x,alpha);
    Forcevalue_filtered.twist.linear.y= exponentialSmoothing(Forcevalue.linear.y,Forcevalue_tmp.twist.linear.y,alpha);
    Forcevalue_filtered.twist.linear.z= exponentialSmoothing(Forcevalue.linear.z,Forcevalue_tmp.twist.linear.z,alpha);
    Forcevalue_filtered.twist.angular.x= exponentialSmoothing(Forcevalue.angular.x,Forcevalue_tmp.twist.angular.x,alpha);
    Forcevalue_filtered.twist.angular.y= exponentialSmoothing(Forcevalue.angular.y,Forcevalue_tmp.twist.angular.y,alpha);
    Forcevalue_filtered.twist.angular.z= exponentialSmoothing(Forcevalue.angular.z,Forcevalue_tmp.twist.angular.z,alpha);
    Forcevalue_tmp = Forcevalue_filtered;

    Forcevalue_filtered.header.stamp =  n->now();

    //std::cout << "filtered force is: " <<endl <<Forcevalue_filtered.twist.linear << std::endl;
    
    //ROTATION AROUND Z AXIS OF 20°//
    Forcevalue_rot_force << Forcevalue_filtered.twist.linear.x,Forcevalue_filtered.twist.linear.y,Forcevalue_filtered.twist.linear.z;
    Forcevalue_rot_torque << Forcevalue_filtered.twist.angular.x,Forcevalue_filtered.twist.angular.y,Forcevalue_filtered.twist.angular.z;
    Forcevalue_rot_force =  RotZ*Forcevalue_rot_force;
    Forcevalue_rot_torque = RotZ*Forcevalue_rot_torque;

    Forcevalue.linear.x = Forcevalue_rot_force[0];
    Forcevalue.linear.y = Forcevalue_rot_force[1];
    Forcevalue.linear.z = Forcevalue_rot_force[2];
    Forcevalue.angular.x = Forcevalue_rot_torque[0];
    Forcevalue.angular.y = Forcevalue_rot_torque[1];
    Forcevalue.angular.z = Forcevalue_rot_torque[2];
   
    // ROS 2 messages have no operator<<; to_yaml (block style) prints the ROS 1 format
    std::cout << "force is: " <<endl <<geometry_msgs::msg::to_yaml(Forcevalue.linear) << std::endl;
  
    chatter_pub->publish(Forcevalue);

    rclcpp::spin_some(n);

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

