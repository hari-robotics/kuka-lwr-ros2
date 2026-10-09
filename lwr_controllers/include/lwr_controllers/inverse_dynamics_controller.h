#ifndef LWR_CONTROLLERS__INVERSE_DYNAMICS_CONTROLLER_H
#define LWR_CONTROLLERS__INVERSE_DYNAMICS_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <boost/thread/condition.hpp>
#include <boost/scoped_ptr.hpp>

namespace lwr_controllers
{

  class InverseDynamicsController : public controller_interface::KinematicChainControllerBase<lwr_controllers::EffortInterface>
  {
  public:

    InverseDynamicsController();
    ~InverseDynamicsController();

    bool configure(lwr_controllers::NodeParameters &n);

    void starting(const rclcpp::Time& time) 
    {
      KDL::SetToZero(torques_);
    }

    void update_command(const rclcpp::Time& time, const rclcpp::Duration& period);

  private:
    rclcpp::SubscriptionBase::SharedPtr ext_wrench_sub_;
    void ext_wrench_cb(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &wrench_msg);

    std::string 
      robot_description_,
      root_name_, 
      tip_name_;
    std::vector<std::string> joint_names_;
      

    unsigned int n_dof_;
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;

    KDL::Wrenches ext_wrenches_;
    KDL::JntArrayAcc joint_states_;
    KDL::JntArray torques_;

  };

}

#endif
