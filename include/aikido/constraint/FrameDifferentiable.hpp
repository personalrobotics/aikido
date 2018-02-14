#ifndef AIKIDO_CONSTRAINT_FRAMEDIFFERENTIABLE_HPP_
#define AIKIDO_CONSTRAINT_FRAMEDIFFERENTIABLE_HPP_

#include <dart/dynamics/dynamics.hpp>
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"
#include "Differentiable.hpp"

#include <Eigen/Dense>

namespace aikido {
namespace constraint {

/// A pose constraint on _jacobianNode's transform
/// w.r.t. MetaSkeletonState of _jacobianNode.
/// _poseConstraint is
///     1) Differentiable
///     2) in SE3.
///     2) constrains _jacobianNode's pose in World Frame.
class FrameDifferentiable : public Differentiable
{
public:
  /// Constructor.
  /// \param _metaSkeletonStateSpace StateSpace whose state
  ///        defines _jacobianNode's transform.
  /// \param _metaskeleton MetaSkeleton to test with
  /// \param _jacobianNode The frame being constrained.
  /// \param _poseConstraint Constraint on _jacobian. This should be
  ///        in SE3.
  FrameDifferentiable(
      statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
      dart::dynamics::MetaSkeletonPtr _metaskeleton,
      dart::dynamics::ConstJacobianNodePtr _jacobianNode,
      DifferentiablePtr _poseConstraint);

  // Documentation inherited.
  std::size_t getConstraintDimension() const override;

  // Documentation inherited.
  void getValue(const statespace::StateSpace::State* _s, Eigen::VectorXd& _out)
      const override;

  /// Get jacobian of poseConstraint w.r.t. generalized coordinates.
  /// \param _s State to be evaluated at.
  /// \param[out] _out _m x numDofs, where m is the number of constraints.
  void getJacobian(
      const statespace::StateSpace::State* _s,
      Eigen::MatrixXd& _out) const override;

  /// Get both Value and Jacobian evaluated at _s.
  /// This is more efficient than calling getValue and getJacobian separately
  /// because this sets MetaSkeleton's position only once.
  /// \param _s State to be evaluated at.
  /// \param[out] _val Value of constraints.
  /// \param[out] _jac Jacobian of constraints.
  void getValueAndJacobian(
      const statespace::StateSpace::State* _s,
      Eigen::VectorXd& _val,
      Eigen::MatrixXd& _jac) const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

private:
  statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  dart::dynamics::ConstJacobianNodePtr mJacobianNode;
  DifferentiablePtr mPoseConstraint;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_FRAMEDIFFERENTIABLE_HPP_
