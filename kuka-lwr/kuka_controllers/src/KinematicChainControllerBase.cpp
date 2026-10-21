#include "../include/KinematicChainControllerBase.h"
// ROS 1: specialisation KinematicChainControllerBase<EffortJointInterface>::getHandles(), which
// also claimed the <joint>_stiffness, <joint>_damping and <joint>_set_point handles. In ROS 2 a
// controller claims exactly the command interfaces it lists; BasicJointController never wrote
// those handles, so only <joint>/effort is claimed (see MIGRATION_NOTES).
