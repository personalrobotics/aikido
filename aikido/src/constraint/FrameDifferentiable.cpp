#include <aikido/constraint/FrameDifferentiable.hpp>
#include <aikido/statespace/SE3.hpp>

namespace aikido {
namespace constraint {


//=============================================================================
FrameDifferentiable::FrameDifferentiable(
  statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
  dart::dynamics::ConstJacobianNodePtr _jacobianNode,
  DifferentiablePtr _poseConstraint)
: mJacobianNode(std::move(_jacobianNode))
, mPoseConstraint(std::move(_poseConstraint))
, mMetaSkeletonStateSpace(std::move(_metaSkeletonStateSpace))
{
  if (!mPoseConstraint)
    throw std::invalid_argument("_poseConstraint is nullptr.");

  if (!mJacobianNode)
    throw std::invalid_argument("_jacobianNode is nullptr.");

  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("_metaSkeletonStateSpace is nullptr.");

  using SE3 = statespace::SE3;

  auto space = dynamic_cast<SE3*>(
    mPoseConstraint->getStateSpace().get());

  if (!space)
    throw std::invalid_argument("_poseConstraint is not in SE3.");

  mMetaSkeleton = mMetaSkeletonStateSpace->getMetaSkeleton();

  if (!mMetaSkeleton)
    throw std::invalid_argument("_metaSkeletonStateSpace does not have skeleton.");

  // TODO: If possible, check that _frame is influenced by at least 
  // one DegreeOfFreedom in the _stateSpace's Skeleton.
}

//=============================================================================
size_t FrameDifferentiable::getConstraintDimension() const
{
  return mPoseConstraint->getConstraintDimension();
}

//=============================================================================
void FrameDifferentiable::getValue(
  const statespace::StateSpace::State* _s,
  Eigen::VectorXd& _out) const
{
  using State = statespace::CartesianProduct::State;
  using SE3State = statespace::SE3::State;

  auto state = static_cast<const State*>(_s);
  
  mMetaSkeletonStateSpace->setState(state);

  SE3State bodyPose(mJacobianNode->getTransform());

  mPoseConstraint->getValue(&bodyPose, _out);

}

//=============================================================================
void FrameDifferentiable::getJacobian(
  const statespace::StateSpace::State* _s,
  Eigen::MatrixXd& _out) const
{
  using State = statespace::CartesianProduct::State;
  using SE3State = statespace::SE3::State;
  using dart::dynamics::MetaSkeletonPtr;

  auto state = static_cast<const State*>(_s);

  mMetaSkeletonStateSpace->setState(state);

  SE3State bodyPose(mJacobianNode->getTransform());
 
  // m x 6 matrix, Jacobian of constraints w.r.t. SE3 pose (se3 tangent vector)
  // where the tangent vector is expressed in world frame.
  Eigen::MatrixXd constraintJac;
  mPoseConstraint->getJacobian(&bodyPose, constraintJac);

  // 6 x numDofs, Jacobian of SE3 pose of body node expressed in World Frame.
  Eigen::MatrixXd skeletonJac = mMetaSkeleton->getWorldJacobian(mJacobianNode);

  // m x numDofs, Jacobian of pose w.r.t. generalized coordinates.
  _out = constraintJac*skeletonJac;

}

//=============================================================================
void FrameDifferentiable::getValueAndJacobian(
    const statespace::StateSpace::State* _s,
    std::pair<Eigen::VectorXd, Eigen::MatrixXd>& _out) const 
{
  using State = statespace::CartesianProduct::State;
  using SE3State = statespace::SE3::State;
  using dart::dynamics::MetaSkeletonPtr;

  auto state = static_cast<const State*>(_s);

  mMetaSkeletonStateSpace->setState(state);

  SE3State bodyPose(mJacobianNode->getTransform());

  mPoseConstraint->getValue(&bodyPose, _out.first);
 
  // m x 6 matrix, Jacobian of constraints w.r.t. SE3 pose (se3 tangent vector)
  // where the tangent vector is expressed in world frame.
  Eigen::MatrixXd constraintJac;
  mPoseConstraint->getJacobian(&bodyPose, constraintJac);

  // 6 x numDofs, Jacobian of SE3 pose of body node expressed in World Frame.
  Eigen::MatrixXd skeletonJac = mMetaSkeleton->getWorldJacobian(mJacobianNode);

  // m x numDofs, Jacobian of pose w.r.t. generalized coordinates.
  _out.second = constraintJac*skeletonJac;
}

//=============================================================================
std::vector<ConstraintType> FrameDifferentiable::getConstraintTypes() const
{
  return mPoseConstraint->getConstraintTypes();
}


//=============================================================================
statespace::StateSpacePtr FrameDifferentiable::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

}
}
