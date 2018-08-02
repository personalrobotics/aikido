#ifndef AIKIDO_PLANNER_DART_UTIL_HPP_
#define AIKIDO_PLANNER_DART_UTIL_HPP_

#include <dart/dynamics/dynamics.hpp>

namespace aikido {
namespace planner {
namespace dart {
namespace util {

/// Returns the direction of an end-effector (along z axis) in the world frame
/// \param[in] body Bodynode for the end-effector
/// \return The direction of the end-effector (z axis of the frame)
Eigen::Vector3d getEndEffectorDirection(
    const ::dart::dynamics::ConstBodyNodePtr& body);

} // namespace util
} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_UTIL_HPP_
