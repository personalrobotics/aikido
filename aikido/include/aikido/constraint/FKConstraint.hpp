#ifndef AIKIDO_CONSTRAINT_FKCONSTRAINT_H
#define AIKIDO_CONSTRAINT_FKCONSTRAINT_H

#include <dart/dynamics/dynamics.h>
#include "Differentiable.hpp"
#include <Eigen/Dense>

namespace aikido {
namespace constraint{

/// A differentiable constraint on _bodyNode's transform
/// w.r.t. the configuration state of _bodyNode's skeleton.
/// _innerConstraint constrains _bodyNode's transform.
class FKConstraint: public Differentiable
{
public:

  FKConstraint(const dart::dynamics::BodyNodePtr& _bodyNode,
                      const DifferentiablePtr& _innerConstraint);

  /// Size of constraints
  size_t getConstraintDimension() const override;

  /// _s should be RealVectorState for generalized coordinates' positions
  Eigen::VectorXd getValue(const state::StatePtr& _s) const override;

  /// Jacobian of constraints at _s. See Jacobian.hpp. 
  state::JacobianPtr getJacobian(const state::StatePtr& _s) const override;

  /// Returns a vector containing each constraint's type.
  std::vector<ConstraintType> getConstraintTypes() const override;

private:
  dart::dynamics::BodyNodePtr mBodyNode;
  DifferentiablePtr mInnerConstraint;
};


} // constraint
} // aikido

#endif