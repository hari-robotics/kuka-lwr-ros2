#ifndef PID_KINEMATIC_CHAIN_CONTROLLER_BASE_H
#define PID_KINEMATIC_CHAIN_CONTROLLER_BASE_H
#include "KinematicChainControllerBase.h"
#include <control_toolbox/pid.hpp>
namespace controller_interface {
template<typename Mode> class PIDKinematicChainControllerBase: public KinematicChainControllerBase<Mode> {
public:
  bool configure(lwr_controllers::NodeParameters &node) override {
    if(!KinematicChainControllerBase<Mode>::configure(node)) return false;
    PIDs_.clear();PIDs_.resize(joint_handles_.size());
    for(size_t i=0;i<joint_handles_.size();++i) {
      lwr_controllers::NodeParameters gains(node,"pid_"+joint_handles_[i].getName());
      double p=0,i_gain=0,d=0,imax=0,imin=0;
      if(!gains.getParam("p",p) || !gains.getParam("i",i_gain) || !gains.getParam("d",d)) return false;
      gains.getParam("i_clamp_max",imax);gains.getParam("i_clamp_min",imin);
      PIDs_[i].initPid(p,i_gain,d,imax,imin);
    }
    return true;
  }
protected:
  using KinematicChainControllerBase<Mode>::kdl_chain_;
  using KinematicChainControllerBase<Mode>::joint_handles_;
  std::vector<control_toolbox::Pid> PIDs_;
  double Kp=0,Ki=0,Kd=0;
};
}
#endif
