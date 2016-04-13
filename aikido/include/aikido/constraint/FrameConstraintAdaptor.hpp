#ifndef AIKIDO_CONSTRAINT_FRAMECONSTRAINTADAPTOR_H
#define AIKIDO_CONSTRAINT_FRAMECONSTRAINTADAPTOR_H

#include <dart/dynamics/dynamics.h>
#include "Differentiable.hpp"
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"

#include <Eigen/Dense>

namespace aikido {
namespace constraint{

/// A differentiable pose constraint on _jacobianNode's transform
/// w.r.t. the configuration state of _jacobianNode's skeleton.
/// _poseConstraint is 
///     1) Differentiable
///     2) in SE3StateSpace.
///     2) constrains _jacobianNode's pose in World Frame.
class FrameConstraintAdaptor: public Differentiable
{
public:

  FrameConstraintAdaptor(
    statespace::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    dart::dynamics::JacobianNodePtr _jacobianNode,
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

  // Documentation inherited. 
  // This is more efficient than calling getValue and getJacobian separately
  // because this sets MetaSkeleton's position only once.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getValueAndJacobian(
    const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

private:
  dart::dynamics::JacobianNodePtr mJacobianNode;
  DifferentiablePtr mPoseConstraint;
  statespace::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
  
};


} // constraint
} // aikido

#endif
