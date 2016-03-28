#ifndef AIKIDO_CONSTRAINT_COMPOUNDCONSTRAINT_H
#define AIKIDO_CONSTRAINT_COMPOUNDCONSTRAINT_H

#include "Differentiable.hpp"

namespace aikido{
namespace constraint{

/// Set of constraints. All constriants should operate on same State.
class CompoundConstraint : public Differentiable
{
public:

  CompoundConstraint(const std::vector<DifferentiablePtr>& _constraints);

  /// Size of constraints
  size_t getConstraintDimension() const override;

  /// Concatenation of values of constraints at _s.
  Eigen::VectorXd getValue(const state::StatePtr& _s) const override;

  /// Concatenation of all Jacobians. 
  /// e.g. if State is SE3, it will return SE3Jacobian (n1+n2...+nk)x6 matrix.
  state::JacobianPtr getJacobian(const state::StatePtr& _s) const override;

  /// Returns a concatenation of all constraint types of _constraints.
  std::vector<ConstraintType> getConstraintTypes() const override;

private:
  std::vector<DifferentiablePtr> mConstraints;

};
}
}

#endif
