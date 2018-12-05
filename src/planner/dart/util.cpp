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

//==============================================================================
::dart::dynamics::MetaSkeletonPtr clone(
    const ::dart::dynamics::MetaSkeletonPtr& metaSkeleton)
{
  using ::dart::dynamics::Group;
  using ::dart::dynamics::BodyNode;

  auto clonedSkeleton = metaSkeleton->getBodyNode(0)->getSkeleton()->clone();

  std::vector<BodyNode*> bodyNodes;
  bodyNodes.reserve(metaSkeleton->getNumBodyNodes());

  for (const auto& bodyNode : metaSkeleton->getBodyNodes())
  {
    auto clonedBodyNode = clonedSkeleton->getBodyNode(bodyNode->getName());
    if (!clonedBodyNode)
    {
      std::stringstream ss;
      ss << bodyNode->getName() << " does not exist in the cloned skel ["
        << clonedSkeleton->getName() << std::endl;
      throw std::runtime_error(ss.str());
    }
    bodyNodes.emplace_back(clonedBodyNode);
  }
  auto group = Group::create(metaSkeleton->getName(), bodyNodes);
}

} // namespace util
} // namespace dart
} // namespace planner
} // namespace aikido
