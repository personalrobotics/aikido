#ifndef AIKIDO_CONSTRAINT_FRAMEPAIRCONSTRAINTADAPTOR_H
#define AIKIDO_CONSTRAINT_FRAMEPAIRCONSTRAINTADAPTOR_H

#include <dart/dynamics/dynamics.h>
#include "Differentiable.hpp"
#include <Eigen/Dense>
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace constraint{

/// A differentiable constraint which constrains relative transform 
/// of jacobianNode1 w.r.t. jacobianNode2.
/// _relPoseConstraint is 
///     1) Differentiable
///     2) in SE3StateSpace.
///     2) constrains _jacobianNode1's pose in jacobianNode2's frame.
class FramePairConstraintAdaptor: public Differentiable
{
public:

  FramePairConstraintAdaptor(
    statespace::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    dart::dynamics::JacobianNodePtr _jacobianNode1,
    dart::dynamics::JacobianNodePtr _jacobianNode2,
    DifferentiablePtr _relPoseConstraint);

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
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getValueAndJacobian(
    const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

private:
  dart::dynamics::JacobianNodePtr mJacobianNode1;
  dart::dynamics::JacobianNodePtr mJacobianNode2;
  DifferentiablePtr mRelPoseConstraint;
  statespace::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
  
};


} // constraint
} // aikido

#endif
