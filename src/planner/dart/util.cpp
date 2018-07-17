#include "aikido/planner/dart/util.hpp"

namespace aikido {
namespace planner {
namespace dart {
namespace util {

//==============================================================================
Eigen::Vector3d getEndEffectorDirection(
    const ::dart::dynamics::ConstBodyNodePtr& body)
{
  const std::size_t zDirection = 2;
  return body->getWorldTransform().linear().col(zDirection).normalized();
}

} // namespace util
} // namespace dart
} // namespace planner
} // namespace aikido
