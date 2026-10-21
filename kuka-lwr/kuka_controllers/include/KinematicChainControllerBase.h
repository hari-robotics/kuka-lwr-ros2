#ifndef KUKA_CONTROLLERS_KINEMATIC_CHAIN_CONTROLLER_BASE_H
#define KUKA_CONTROLLERS_KINEMATIC_CHAIN_CONTROLLER_BASE_H

// ROS 2: the ROS 1 kuka_controllers/include/KinematicChainControllerBase.h was identical
// (whitespace aside) to lwr_controllers/KinematicChainControllerBase.h and defined the same
// template controller_interface::KinematicChainControllerBase. Both plugin libraries are
// loaded by the same controller_manager, so the ported lwr_controllers base is used instead of
// a second definition of the same template.
#include <lwr_controllers/KinematicChainControllerBase.h>

#endif
