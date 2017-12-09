#include "BodyNodePoseVectorFieldConstraint.hpp"
#include <aikido/constraint/JointStateSpaceHelpers.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {
namespace detail {

//==============================================================================
BodyNodePoseVectorFieldConstraint::BodyNodePoseVectorFieldConstraint(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    aikido::constraint::TestablePtr stateConstraint)
  : mStateSpace(stateSpace), mStateConstraint(stateConstraint)
{
  mStateSpaceBoundConstraint
      = aikido::constraint::createTestableBounds(stateSpace);
}

//==============================================================================
bool BodyNodePoseVectorFieldConstraint::isSatisfied(
    const statespace::StateSpace::State* state) const
{
  if (mStateConstraint)
  {
    if (!mStateConstraint->isSatisfied(state))
    {
      return false;
    }
  }
  if (mStateSpaceBoundConstraint)
  {
    if (!mStateSpaceBoundConstraint->isSatisfied(state))
    {
      return false;
    }
  }
  return true;
}

//==============================================================================
statespace::StateSpacePtr BodyNodePoseVectorFieldConstraint::getStateSpace()
    const
{
  return mStateSpace;
}

} // namespace detail
} // namespace vectorfield
} // namespace planner
} // namespace aikido
