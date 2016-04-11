#include <aikido/constraint/FrameConstraintAdaptor.hpp>

namespace aikido {
namespace constraint {


//=============================================================================
FrameConstraintAdaptor::FrameConstraintAdaptor(
  statespace::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
  dart::dynamics::BodyNodePtr& _bodyNode,
  TSRPtr& _poseConstraint)
: mBodyNode(_bodyNode)
, mPoseConstraint(_poseConstraint)
, mMetaSkeletonStateSpace(_metaSkeletonStateSpace)
{
  if (!mPoseConstraint)
    throw std::invalid_argument("_poseConstraint is nullptr.");

  if (!mBodyNode)
    throw std::invalid_argument("_bodyNode is nullptr.");

  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("_metaSkeletonStateSpace is nullptr.");
}


//=============================================================================
size_t FrameConstraintAdaptor::getConstraintDimension() const
{
  return mPoseConstraint->getConstraintDimension();
}


//=============================================================================
Eigen::VectorXd FrameConstraintAdaptor::getValue(
  const statespace::StateSpace::State* _s) const
{
  using State = statespace::CompoundStateSpace::State;
  using SE3State = statespace::SE3StateSpace::State;

  auto state = static_cast<const State*>(_s);
  
  mMetaSkeletonStateSpace->setStateOnMetaSkeleton(state);

  SE3State bodyPose(mBodyNode->getTransform());

  return mPoseConstraint->getValue(&bodyPose);

}


//=============================================================================
Eigen::MatrixXd FrameConstraintAdaptor::getJacobian(
  const statespace::StateSpace::State* _s) const
{
  using State = statespace::CompoundStateSpace::State;
  using SE3State = statespace::SE3StateSpace::State;
  using dart::dynamics::MetaSkeletonPtr;

  auto state = static_cast<const State*>(_s);

  mMetaSkeletonStateSpace->setStateOnMetaSkeleton(state);

  SE3State bodyPose(mBodyNode->getTransform());
 
  Eigen::MatrixXd constraintJac = mPoseConstraint->getJacobian(&bodyPose);

  MetaSkeletonPtr metaSkeleton = mMetaSkeletonStateSpace->getMetaSkeleton();
  return constraintJac*metaSkeleton->getJacobian(mBodyNode);

}


//=============================================================================
std::vector<ConstraintType> FrameConstraintAdaptor::getConstraintTypes() const
{
  return mPoseConstraint->getConstraintTypes();
}


//=============================================================================
statespace::StateSpacePtr FrameConstraintAdaptor::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

}
}
