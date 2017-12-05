#include <aikido/planner/vectorfield/BodyNodePoseVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
BodyNodePoseVectorField::BodyNodePoseVectorField(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    dart::dynamics::BodyNodePtr bodyNode)
  : MetaSkeletonStateSpaceVectorField(stateSpace), mBodyNode(bodyNode)
{
  // Do nothing
}

//==============================================================================
dart::dynamics::BodyNodePtr BodyNodePoseVectorField::getBodyNode()
{
  return mBodyNode;
}

//==============================================================================
dart::dynamics::ConstBodyNodePtr BodyNodePoseVectorField::getBodyNode() const
{
  return mBodyNode;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
