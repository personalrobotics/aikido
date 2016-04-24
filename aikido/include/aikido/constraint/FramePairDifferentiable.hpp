#ifndef AIKIDO_CONSTRAINT_FRAMEPAIRDIFFERENTIABLE_HPP_
#define AIKIDO_CONSTRAINT_FRAMEPAIRDIFFERENTIABLE_HPP_

#include <dart/dynamics/dynamics.h>
#include "Differentiable.hpp"
#include <Eigen/Dense>
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace constraint{

/// A differentiable constraint which constrains relative transform 
/// of jacobianNodeTarget w.r.t. jacobianNodeBase.
/// _relPoseConstraint is 
///     1) Differentiable
///     2) in SE3.
///     2) constrains _jacobianNodeTarget's pose in jacobianNodeBase's frame.
class FramePairDifferentiable: public Differentiable
{
public:

  /// Constructor.
  /// \param _metaSkeletonStateSpace StateSpace whose states define 
  ///        _jacobianNodeTarget and _jacobianNodeBase's relative transform.
  /// \param _jacobianNodeTarget The frame whose relative transform w.r.t. 
  ///        _jacobianNodeBase is being constrained.
  /// \param _jacobianNodeBase The base frame for this constraint.
  /// \param _relPoseConstraint Relative pose constraint on _jacobianNodeTarget
  ///        w.r.t. _jacobianNodeBase. 
  FramePairDifferentiable(
    statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    dart::dynamics::ConstJacobianNodePtr _jacobianNodeTarget,
    dart::dynamics::ConstJacobianNodePtr _jacobianNodeBase,
    DifferentiablePtr _relPoseConstraint);

  // Documentation inherited.
  size_t getConstraintDimension() const override;
  
  // Documentation inherited.
  void getValue(
    const statespace::StateSpace::State* _s,
    Eigen::VectorXd& _out) const override;

  // Documentation inherited.
  void getJacobian(
    const statespace::StateSpace::State* _s,
    Eigen::MatrixXd& _out) const override;

  // Documentation inherited.
  void getValueAndJacobian(
    const statespace::StateSpace::State* _s,
    std::pair<Eigen::VectorXd, Eigen::MatrixXd>& _out) const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

private:
  dart::dynamics::ConstJacobianNodePtr mJacobianNode1;
  dart::dynamics::ConstJacobianNodePtr mJacobianNode2;
  DifferentiablePtr mRelPoseConstraint;
  statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;

};


} // constraint
} // aikido

#endif
