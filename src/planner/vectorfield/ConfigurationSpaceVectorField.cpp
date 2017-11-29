#include <memory>
#include <dart/dynamics/BodyNode.hpp>
#include <aikido/planner/vectorfield/ConfigurationSpaceVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

ConfigurationSpaceVectorField::ConfigurationSpaceVectorField(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    dart::dynamics::BodyNodePtr _bodyNode)
  : mStateSpace(_stateSpace)
  , mMetaSkeleton(_stateSpace->getMetaSkeleton())
  , mBodyNode(_bodyNode)
{
  // Do nothing
}

aikido::statespace::dart::MetaSkeletonStateSpacePtr
ConfigurationSpaceVectorField::getMetaSkeletonStateSpace() const
{
  return mStateSpace;
}

/// Meta skeleton
dart::dynamics::MetaSkeletonPtr ConfigurationSpaceVectorField::getMetaSkeleton()
    const
{
  return mMetaSkeleton;
}

/// Body node of end-effector
dart::dynamics::BodyNodePtr ConfigurationSpaceVectorField::getBodyNode() const
{
  return mBodyNode;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
