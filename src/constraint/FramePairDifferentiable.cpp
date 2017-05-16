#include <aikido/constraint/FramePairDifferentiable.hpp>
#include <aikido/statespace/SE3.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
FramePairDifferentiable::FramePairDifferentiable(
    statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    dart::dynamics::ConstJacobianNodePtr _jacobianNode1,
    dart::dynamics::ConstJacobianNodePtr _jacobianNode2,
    DifferentiablePtr _relPoseConstraint)
  : mJacobianNode1(std::move(_jacobianNode1))
  , mJacobianNode2(std::move(_jacobianNode2))
  , mRelPoseConstraint(std::move(_relPoseConstraint))
  , mMetaSkeletonStateSpace(std::move(_metaSkeletonStateSpace))
{
  if (!mRelPoseConstraint)
    throw std::invalid_argument("_relPoseConstraint is nullptr.");

  if (!mJacobianNode1)
    throw std::invalid_argument("_jacobianNode1 is nullptr.");

  if (!mJacobianNode2)
    throw std::invalid_argument("_jacobianNode2 is nullptr.");

  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("_metaSkeletonStateSpace is nullptr.");

  using SE3 = statespace::SE3;

  auto space = dynamic_cast<SE3*>(mRelPoseConstraint->getStateSpace().get());

  if (!space)
    throw std::invalid_argument("_relPoseConstraint is not in SE3.");

  mMetaSkeleton = mMetaSkeletonStateSpace->getMetaSkeleton();

  if (!mMetaSkeleton)
  {
    throw std::invalid_argument(
        "_metaSkeletonStateSpace does not have skeleton.");
  }

  // TODO: check that _jacobianNode1 and _jacobianNode2
  // are influenced by at least one DegreeOfFreedom of _metaSkeletonStateSpace.
}

//=============================================================================
size_t FramePairDifferentiable::getConstraintDimension() const
{
  return mRelPoseConstraint->getConstraintDimension();
}

//=============================================================================
void FramePairDifferentiable::getValue(
    const statespace::StateSpace::State* _s, Eigen::VectorXd& _out) const
{
  using State = statespace::CartesianProduct::State;
  using SE3State = statespace::SE3::State;

  auto state = static_cast<const State*>(_s);

  mMetaSkeletonStateSpace->setState(state);

  // Relative transform of mJacobianNode1 w.r.t. mJacobianNode2,
  // expressed in mJacobianNode2 frame.
  SE3State relativeTransform(
      mJacobianNode1->getTransform(mJacobianNode2, mJacobianNode2));

  mRelPoseConstraint->getValue(&relativeTransform, _out);
}

//=============================================================================
void FramePairDifferentiable::getJacobian(
    const statespace::StateSpace::State* _s, Eigen::MatrixXd& _out) const
{
  using State = statespace::CartesianProduct::State;
  using SE3State = statespace::SE3::State;
  using dart::dynamics::MetaSkeletonPtr;

  auto state = static_cast<const State*>(_s);

  mMetaSkeletonStateSpace->setState(state);

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

//=============================================================================
void FramePairDifferentiable::getValueAndJacobian(
    const statespace::StateSpace::State* _s,
    Eigen::VectorXd& _val,
    Eigen::MatrixXd& _jac) const
{
  using State = statespace::CartesianProduct::State;
  using SE3State = statespace::SE3::State;
  using dart::dynamics::MetaSkeletonPtr;

  auto state = static_cast<const State*>(_s);

  mMetaSkeletonStateSpace->setState(state);

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

//=============================================================================
std::vector<ConstraintType> FramePairDifferentiable::getConstraintTypes() const
{
  return mRelPoseConstraint->getConstraintTypes();
}

//=============================================================================
statespace::StateSpacePtr FramePairDifferentiable::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

} // namespace constraint
} // namespace aikido
