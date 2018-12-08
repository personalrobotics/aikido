#include "aikido/planner/dart/util.hpp"

namespace aikido {
namespace planner {
namespace dart {
namespace util {

using ::dart::dynamics::Group;
using ::dart::dynamics::BodyNode;

//==============================================================================
Eigen::Vector3d getEndEffectorDirection(
    const ::dart::dynamics::ConstBodyNodePtr& body)
{
  const std::size_t zDirection = 2;
  return body->getWorldTransform().linear().col(zDirection).normalized();
}

//==============================================================================
::dart::dynamics::MetaSkeletonPtr clone(
    const ::dart::dynamics::MetaSkeletonPtr& metaSkeleton)
{
  return metaSkeleton->cloneMetaSkeleton();
}

} // namespace util
} // namespace dart
} // namespace planner
} // namespace aikido
