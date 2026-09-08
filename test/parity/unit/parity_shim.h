#ifndef GBP_PARITY_SHIM_H_
#define GBP_PARITY_SHIM_H_

// Selects the side-specific shim. The build scripts define exactly one of
// GBP_PARITY_ROS1 / GBP_PARITY_ROS2; nothing else in the suite is allowed to
// test for them.

#if defined(GBP_PARITY_ROS1)
#include "parity_shim_ros1.h"
#elif defined(GBP_PARITY_ROS2)
#include "parity_shim_ros2.h"
#else
#error "define GBP_PARITY_ROS1 or GBP_PARITY_ROS2"
#endif

#endif  // GBP_PARITY_SHIM_H_
