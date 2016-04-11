#ifndef AIKIDO_CONSTRAINT_FRAMEPAIRCONSTRAINTADAPTOR_H
#define AIKIDO_CONSTRAINT_FRAMEPAIRCONSTRAINTADAPTOR_H

#include <dart/dynamics/dynamics.h>
#include "Differentiable.hpp"
#include <Eigen/Dense>
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"
#include "TSR.hpp"

namespace aikido {
namespace constraint{

/// A differentiable constraint.
/// Constrains relative transform of bodynode1 w.r.t. bodynode2.
class FramePairConstraintAdaptor: public Differentiable
{
public:

  FramePairConstraintAdaptor(
    statespace::MetaSkeletonStateSpacePtr& _metaSkeletonStateSpace,
    dart::dynamics::BodyNodePtr& _bodyNode1,
    dart::dynamics::BodyNodePtr& _bodyNode2,
    TSRPtr& _poseConstraint);

  // Documentation inherited.
  size_t getConstraintDimension() const override;
  
  // Documentation inherited.
  Eigen::VectorXd getValue(
    const statespace::StateSpace::State* _s) const override; 

  // Documentation inherited.
  Eigen::MatrixXd getJacobian(
    const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

private:
  dart::dynamics::BodyNodePtr mBodyNode1;
  dart::dynamics::BodyNodePtr mBodyNode2;
  TSRPtr mPoseConstraint;
  statespace::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
  
};


} // constraint
} // aikido

#endif
