#ifndef AIKIDO_CONSTRAINT_FRAMECONSTRAINTADAPTOR_HPP_
#define AIKIDO_CONSTRAINT_FRAMECONSTRAINTADAPTOR_HPP_

#include <dart/dynamics/dynamics.h>
#include "Differentiable.hpp"
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"

#include <Eigen/Dense>

namespace aikido {
namespace constraint{

/// A pose constraint on _jacobianNode's transform
/// w.r.t. MetaSkeletonState of _jacobianNode.
/// _poseConstraint is 
///     1) Differentiable
///     2) in SE3.
///     2) constrains _jacobianNode's pose in World Frame.
class FrameConstraintAdaptor: public Differentiable
{
public:

  /// Constructor.
  /// \param _metaSkeletonStateSpace StateSpace whose state
  ///        defines _jacobianNode's transform.
  /// \param _jacobianNode The frame being constrained.
  /// \param _poseConstraint Constraint on _jacobian. This should be 
  ///        in SE3.
  FrameConstraintAdaptor(
    statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    dart::dynamics::ConstJacobianNodePtr _jacobianNode,
    DifferentiablePtr _poseConstraint);

  // Documentation inherited.
  size_t getConstraintDimension() const override;

  // Documentation inherited.
  Eigen::VectorXd getValue(
    const statespace::StateSpace::State* _s) const override; 

  //  m x numDofs, where m is the number of constraints. 
  //  Jacobian of poseConstraint w.r.t. generalized coordinates.
  Eigen::MatrixXd getJacobian(
    const statespace::StateSpace::State* _s) const override;

  /// Returns both Value and Jacobian.
  /// This is more efficient than calling getValue and getJacobian separately
  /// because this sets MetaSkeleton's position only once.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getValueAndJacobian(
    const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

private:
  dart::dynamics::ConstJacobianNodePtr mJacobianNode;
  DifferentiablePtr mPoseConstraint;
  statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;

  
};


} // constraint
} // aikido

#endif
