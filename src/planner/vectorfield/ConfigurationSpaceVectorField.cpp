#include <dart/dynamics/BodyNode.hpp>
#include <aikido/planner/vectorfield/ConfigurationSpaceVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
ConfigurationSpaceVectorField::ConfigurationSpaceVectorField(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    dart::dynamics::BodyNodePtr _bodyNode)
  : mStateSpace(_stateSpace)
  , mMetaSkeleton(_stateSpace->getMetaSkeleton())
  , mBodyNode(_bodyNode)
{
  // Do nothing
}

//==============================================================================
aikido::statespace::dart::MetaSkeletonStateSpacePtr
ConfigurationSpaceVectorField::getMetaSkeletonStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
dart::dynamics::MetaSkeletonPtr ConfigurationSpaceVectorField::getMetaSkeleton()
{
  return mMetaSkeleton;
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr
ConfigurationSpaceVectorField::getMetaSkeleton() const
{
  return mMetaSkeleton;
}

//==============================================================================
dart::dynamics::BodyNodePtr ConfigurationSpaceVectorField::getBodyNode()
{
  return mBodyNode;
}

//==============================================================================
dart::dynamics::ConstBodyNodePtr ConfigurationSpaceVectorField::getBodyNode()
    const
{
  return mBodyNode;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
