#ifndef AIKIDO_CONSTRAINT_FRAMECONSTRAINTADAPTOR_H
#define AIKIDO_CONSTRAINT_FRAMECONSTRAINTADAPTOR_H

#include <dart/dynamics/dynamics.h>
#include "Differentiable.hpp"
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"
#include "TSR.hpp"

#include <Eigen/Dense>

namespace aikido {
namespace constraint{

/// A differentiable constraint on _bodyNode's transform
/// w.r.t. the configuration state of _bodyNode's skeleton.
/// _innerConstraint constrains _bodyNode's transform.
class FrameConstraintAdaptor: public Differentiable
{
public:

  FrameConstraintAdaptor(
    statespace::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    dart::dynamics::BodyNodePtr& _bodyNode,
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
  dart::dynamics::BodyNodePtr mBodyNode;
  TSRPtr mPoseConstraint;
  statespace::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
  
};


} // constraint
} // aikido

#endif
