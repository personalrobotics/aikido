#include <aikido/constraint/FramePairConstraintAdaptor.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>

namespace aikido {
namespace constraint {


//=============================================================================
FramePairConstraintAdaptor::FramePairConstraintAdaptor(
    statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    dart::dynamics::JacobianNodePtr _jacobianNode1,
    dart::dynamics::JacobianNodePtr _jacobianNode2,
    DifferentiablePtr _relPoseConstraint)
: mJacobianNode1(_jacobianNode1)
, mJacobianNode2(_jacobianNode2)
, mRelPoseConstraint(_relPoseConstraint)
, mMetaSkeletonStateSpace(_metaSkeletonStateSpace)
{
  if (!mRelPoseConstraint)
    throw std::invalid_argument("_relPoseConstraint is nullptr.");

  if (!mJacobianNode1)
    throw std::invalid_argument("_bodyNode1 is nullptr.");

  if (!mJacobianNode2)
    throw std::invalid_argument("_bodyNode2 is nullptr.");

  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("_metaSkeletonStateSpace is nullptr.");

  using SE3StateSpace = statespace::SE3StateSpace;

  auto space = dynamic_cast<SE3StateSpace*>(
    mRelPoseConstraint->getStateSpace().get());

  if (!space)
  {
    throw std::invalid_argument("_relPoseConstraint is not in SE3StateSpace.");
  }
}


//=============================================================================
size_t FramePairConstraintAdaptor::getConstraintDimension() const
{
  return mRelPoseConstraint->getConstraintDimension();
}


//=============================================================================
Eigen::VectorXd FramePairConstraintAdaptor::getValue(
  const statespace::StateSpace::State* _s) const
{
  using State = statespace::CompoundStateSpace::State;
  using SE3State = statespace::SE3StateSpace::State;

  auto state = static_cast<const State*>(_s);
  
  mMetaSkeletonStateSpace->setStateOnMetaSkeleton(state);

  // Relative transform of mJacobianNode1 w.r.t. mJacobianNode2,
  // expressed in mJacobianNode2 frame.
  SE3State relativeTransform(
    mJacobianNode1->getTransform(mJacobianNode2, mJacobianNode2));

  return mRelPoseConstraint->getValue(&relativeTransform);
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

  // Relative transform of mJacobianNode1 w.r.t. mJacobianNode2,
  // expressed in mJacobianNode2's frame.
  SE3State relTransform(
    mJacobianNode1->getTransform(mJacobianNode2, mJacobianNode2));
 
  // m x 6 matrix, Jacobian of constraints w.r.t. SE3 pose (se3 tangent vector)
  // where the tangent vector is expressed in mJacobianNode2's frame.
  Eigen::MatrixXd constraintJac = mRelPoseConstraint->getJacobian(&relTransform);

  MetaSkeletonPtr metaSkeleton = mMetaSkeletonStateSpace->getMetaSkeleton();

  // 6 x numDofs,
  // Jacobian of relative transform expressed in mJacobianNode2's Frame.
  Eigen::MatrixXd skeletonJac = 
    metaSkeleton->getJacobian(mJacobianNode1, mJacobianNode2)
    - metaSkeleton->getJacobian(mJacobianNode2, mJacobianNode2);

  // m x numDofs,
  // Jacobian of relative pose constraint w.r.t generalized coordinates.
  return constraintJac*skeletonJac;
}


//=============================================================================
std::pair<Eigen::VectorXd, Eigen::MatrixXd> FramePairConstraintAdaptor::getValueAndJacobian(
    const statespace::StateSpace::State* _s) const
{
  using State = statespace::CompoundStateSpace::State;
  using SE3State = statespace::SE3StateSpace::State;
  using dart::dynamics::MetaSkeletonPtr;

  auto state = static_cast<const State*>(_s);

  mMetaSkeletonStateSpace->setStateOnMetaSkeleton(state);

  // Relative transform of mJacobianNode1 w.r.t. mJacobianNode2,
  // expressed in mJacobianNode2's frame.
  SE3State relTransform(
    mJacobianNode1->getTransform(mJacobianNode2, mJacobianNode2));

  Eigen::VectorXd value = mRelPoseConstraint->getValue(&relTransform);

  // m x 6 matrix, Jacobian of constraints w.r.t. SE3 pose (se3 tangent vector)
  // where the tangent vector is expressed in mJacobianNode2's frame.
  Eigen::MatrixXd constraintJac = mRelPoseConstraint->getJacobian(&relTransform);

  MetaSkeletonPtr metaSkeleton = mMetaSkeletonStateSpace->getMetaSkeleton();

  // 6 x numDofs,
  // Jacobian of relative transform expressed in mJacobianNode2's Frame.
  Eigen::MatrixXd skeletonJac = 
    metaSkeleton->getJacobian(mJacobianNode1, mJacobianNode2)
    - metaSkeleton->getJacobian(mJacobianNode2, mJacobianNode2);

  // m x numDofs,
  // Jacobian of relative pose constraint w.r.t generalized coordinates.
  Eigen::MatrixXd jacobian = constraintJac*skeletonJac;

  return std::make_pair(value, jacobian);

}


//=============================================================================
std::vector<ConstraintType> FramePairConstraintAdaptor::getConstraintTypes() const
{
  return mRelPoseConstraint->getConstraintTypes();
}


//=============================================================================
statespace::StateSpacePtr FramePairConstraintAdaptor::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

}
}
