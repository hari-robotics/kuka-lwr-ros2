#ifndef HARDWARE_INTERFACE_CARTESIAN_STATE_INTERFACE_H
#define HARDWARE_INTERFACE_CARTESIAN_STATE_INTERFACE_H
#include <hardware_interface/handle.hpp>
namespace hardware_interface {
// Cartesian scalar resources now use ordinary named ros2_control interfaces.
using CartesianStateHandle = StateInterface;
}
#endif
