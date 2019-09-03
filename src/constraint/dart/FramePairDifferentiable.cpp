#include "aikido/constraint/dart/FramePairDifferentiable.hpp"

#include "aikido/statespace/SE3.hpp"

namespace aikido {
namespace constraint {
namespace dart {

//==============================================================================
FramePairDifferentiable::FramePairDifferentiable(
    statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    ::dart::dynamics::MetaSkeletonPtr _metaskeleton,
    ::dart::dynamics::ConstJacobianNodePtr _jacobianNode1,
    ::dart::dynamics::ConstJacobianNodePtr _jacobianNode2,
    DifferentiablePtr _relPoseConstraint)
  : mMetaSkeletonStateSpace(std::move(_metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(_metaskeleton))
  , mJacobianNode1(std::move(_jacobianNode1))
  , mJacobianNode2(std::move(_jacobianNode2))
  , mRelPoseConstraint(std::move(_relPoseConstraint))
{
  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("_metaSkeletonStateSpace is nullptr.");

  // TODO: Check compatibility between MetaSkeleton and MetaSkeletonStateSpace
  if (!mMetaSkeleton)
    throw std::invalid_argument("_metaskeleton is nullptr.");

  if (!mRelPoseConstraint)
    throw std::invalid_argument("_relPoseConstraint is nullptr.");

  if (!mJacobianNode1)
    throw std::invalid_argument("_jacobianNode1 is nullptr.");

  if (!mJacobianNode2)
    throw std::invalid_argument("_jacobianNode2 is nullptr.");

  using SE3 = statespace::SE3;

  auto space
      = dynamic_cast<const SE3*>(mRelPoseConstraint->getStateSpace().get());

  if (!space)
    throw std::invalid_argument("_relPoseConstraint is not in SE3.");

  // TODO: check that _jacobianNode1 and _jacobianNode2
  // are influenced by at least one DegreeOfFreedom of _metaSkeletonStateSpace.
}

//==============================================================================
std::size_t FramePairDifferentiable::getConstraintDimension() const
{
  return mRelPoseConstraint->getConstraintDimension();
}

//==============================================================================
void FramePairDifferentiable::getValue(
    const statespace::StateSpace::State* _s, Eigen::VectorXd& _out) const
{
  using State = statespace::CartesianProduct::State;
  using SE3State = statespace::SE3::State;

  auto state = static_cast<const State*>(_s);

  mMetaSkeletonStateSpace->setState(mMetaSkeleton.get(), state);

  // Relative transform of mJacobianNode1 w.r.t. mJacobianNode2,
  // expressed in mJacobianNode2 frame.
  SE3State relativeTransform(
      mJacobianNode1->getTransform(mJacobianNode2, mJacobianNode2));

  mRelPoseConstraint->getValue(&relativeTransform, _out);
}

//==============================================================================
void FramePairDifferentiable::getJacobian(
    const statespace::StateSpace::State* _s, Eigen::MatrixXd& _out) const
{
  using State = statespace::CartesianProduct::State;
  using SE3State = statespace::SE3::State;

  auto state = static_cast<const State*>(_s);

  mMetaSkeletonStateSpace->setState(mMetaSkeleton.get(), state);

  // Relative transform of mJacobianNode1 w.r.t. mJacobianNode2,
  // expressed in mJacobianNode2's frame.
  SE3State relTransform(
      mJacobianNode1->getTransform(mJacobianNode2, mJacobianNode2));

  // m x 6 matrix, Jacobian of constraints w.r.t. SE3 pose (se3 tangent vector)
  // where the tangent vector is expressed in mJacobianNode2's frame.
  Eigen::MatrixXd constraintJac;
  mRelPoseConstraint->getJacobian(&relTransform, constraintJac);

  // 6 x numDofs,
  // Jacobian of relative transform expressed in mJacobianNode2's Frame.
  Eigen::MatrixXd skeletonJac
      = mMetaSkeleton->getJacobian(mJacobianNode1, mJacobianNode2)
        - mMetaSkeleton->getJacobian(mJacobianNode2, mJacobianNode2);

  // m x numDofs,
  // Jacobian of relative pose constraint w.r.t generalized coordinates.
  _out = constraintJac * skeletonJac;
}

//==============================================================================
void FramePairDifferentiable::getValueAndJacobian(
    const statespace::StateSpace::State* _s,
    Eigen::VectorXd& _val,
    Eigen::MatrixXd& _jac) const
{
  using State = statespace::CartesianProduct::State;
  using SE3State = statespace::SE3::State;

  auto state = static_cast<const State*>(_s);

  mMetaSkeletonStateSpace->setState(mMetaSkeleton.get(), state);

  // Relative transform of mJacobianNode1 w.r.t. mJacobianNode2,
  // expressed in mJacobianNode2's frame.
  SE3State relTransform(
      mJacobianNode1->getTransform(mJacobianNode2, mJacobianNode2));

  mRelPoseConstraint->getValue(&relTransform, _val);

  // m x 6 matrix, Jacobian of constraints w.r.t. SE3 pose (se3 tangent vector)
  // where the tangent vector is expressed in mJacobianNode2's frame.
  Eigen::MatrixXd constraintJac;
  mRelPoseConstraint->getJacobian(&relTransform, constraintJac);

  // 6 x numDofs,
  // Jacobian of relative transform expressed in mJacobianNode2's Frame.
  Eigen::MatrixXd skeletonJac
      = mMetaSkeleton->getJacobian(mJacobianNode1, mJacobianNode2)
        - mMetaSkeleton->getJacobian(mJacobianNode2, mJacobianNode2);

  // m x numDofs,
  // Jacobian of relative pose constraint w.r.t generalized coordinates.
  _jac = constraintJac * skeletonJac;
}

//==============================================================================
std::vector<ConstraintType> FramePairDifferentiable::getConstraintTypes() const
{
  return mRelPoseConstraint->getConstraintTypes();
}

//==============================================================================
statespace::ConstStateSpacePtr FramePairDifferentiable::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

} // namespace dart
} // namespace constraint
} // namespace aikido
