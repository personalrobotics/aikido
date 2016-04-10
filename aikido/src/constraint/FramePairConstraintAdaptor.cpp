#include <aikido/constraint/FramePairConstraintAdaptor.hpp>

namespace aikido {
namespace constraint {


//=============================================================================
FramePairConstraintAdaptor::FramePairConstraintAdaptor(
    statespace::MetaSkeletonStateSpacePtr& _metaSkeletonStateSpace,
    dart::dynamics::BodyNodePtr& _bodyNode1,
    dart::dynamics::BodyNodePtr& _bodyNode2,
    TSRPtr& _poseConstraint)
: mBodyNode1(_bodyNode1)
, mBodyNode2(_bodyNode2)
, mPoseConstraint(_poseConstraint)
, mMetaSkeletonStateSpace(_metaSkeletonStateSpace)
{
  if (!mPoseConstraint)
    throw std::invalid_argument("_poseConstraint is nullptr.");

  if (!mBodyNode1)
    throw std::invalid_argument("_bodyNode1 is nullptr.");

  if (!mBodyNode2)
    throw std::invalid_argument("_bodyNode2 is nullptr.");

  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("_metaSkeletonStateSpace is nullptr.");
}


//=============================================================================
size_t FramePairConstraintAdaptor::getConstraintDimension() const
{
  return mPoseConstraint->getConstraintDimension();
}


//=============================================================================
Eigen::VectorXd FramePairConstraintAdaptor::getValue(
  const statespace::StateSpace::State* _s) const
{
  using State = statespace::CompoundStateSpace::State;
  using SE3State = statespace::SE3StateSpace::State;

  auto state = static_cast<const State*>(_s);
  
  mMetaSkeletonStateSpace->setStateOnMetaSkeleton(state);

  SE3State relativeTransform(mBodyNode1->getTransform(mBodyNode2, mBodyNode2));

  return mPoseConstraint->getValue(&relativeTransform);

}


//=============================================================================
Eigen::MatrixXd FramePairConstraintAdaptor::getJacobian(
  const statespace::StateSpace::State* _s) const
{
  using State = statespace::CompoundStateSpace::State;
  using SE3State = statespace::SE3StateSpace::State;
  using dart::dynamics::MetaSkeletonPtr;

  auto state = static_cast<const State*>(_s);

  mMetaSkeletonStateSpace->setStateOnMetaSkeleton(state);

  SE3State relTransform(mBodyNode1->getTransform(mBodyNode2, mBodyNode2));
 
  Eigen::MatrixXd constraintJac = mPoseConstraint->getJacobian(&relTransform);

  MetaSkeletonPtr metaSkeleton = mMetaSkeletonStateSpace->getMetaSkeleton();
  return constraintJac*(metaSkeleton->getJacobian(mBodyNode1, mBodyNode2)
                        - metaSkeleton->getJacobian(mBodyNode2, mBodyNode2));

}


//=============================================================================
std::vector<ConstraintType> FramePairConstraintAdaptor::getConstraintTypes() const
{
  return mPoseConstraint->getConstraintTypes();
}


//=============================================================================
statespace::StateSpacePtr FramePairConstraintAdaptor::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

}
}
