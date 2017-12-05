#include <dart/dynamics/BodyNode.hpp>
#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/planner/vectorfield/MetaSkeletonStateSpaceVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
MetaSkeletonStateSpaceVectorField::MetaSkeletonStateSpaceVectorField(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace)
  : VectorField(stateSpace)
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(stateSpace->getMetaSkeleton())
{
  if (mMetaSkeleton->getPositionUpperLimits()
      == mMetaSkeleton->getPositionLowerLimits())
  {
    throw std::invalid_argument("State space volume zero");
  }
  if (mMetaSkeleton->getVelocityUpperLimits()
      == mMetaSkeleton->getVelocityLowerLimits())
  {
    throw std::invalid_argument("Velocity space volume zero");
  }
}

//==============================================================================
aikido::constraint::TestablePtr
MetaSkeletonStateSpaceVectorField::getBoundTestable() const
{
  return aikido::constraint::createTestableBounds(mMetaSkeletonStateSpace);
}

//==============================================================================
aikido::statespace::dart::MetaSkeletonStateSpacePtr
MetaSkeletonStateSpaceVectorField::getMetaSkeletonStateSpace()
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
MetaSkeletonStateSpaceVectorField::getMetaSkeletonStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
dart::dynamics::MetaSkeletonPtr
MetaSkeletonStateSpaceVectorField::getMetaSkeleton()
{
  return mMetaSkeleton;
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr
MetaSkeletonStateSpaceVectorField::getMetaSkeleton() const
{
  return mMetaSkeleton;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
