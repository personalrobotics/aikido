#ifndef AIKIDO_CONSTRAINT_FKCONSTRAINTADAPTOR_H
#define AIKIDO_CONSTRAINT_FKCONSTRAINTADAPTOR_H

#include "ConstraintType.hpp"
#include "Differentiable.hpp"
#include <Eigen/Dense>

namespace aikido {
namespace constraint{

/// _bodyNode's transform must satisfy _innerConstraint.
class FKConstraintAdaptor: public Differentiable
{
public:

  FKConstraintAdaptor(const dart::dynamics::BodyNodePtr& _bodyNode,
                      const DifferentiablePtr& _innerConstraint);

  /// Size of constraints
  size_t getConstraintDimension() const override;


  /// _s should be RealVectorState for generalized coordinates' positions
  Eigen::VectorXd getValue(const state::CompoundState& _s) const override;


  /// Jacobian of constraints at _s.
  /// For SO3 state, returns mx3 matrix.
  /// For SO2 state, returns mx1 matrix.
  /// For vector<n> state, returns mxn matrix.
  /// For compound state with k components, returns k matrices 
  /// where ith value correcsponds to Jacobian at ith component.
  vector<Eigen::MatrixXd> getJacobian(
    const state::CompoundState& _s) const override;


  /// Returns a vector containing each constraint's type.
  std::vector<ConstraintType> getConstraintTypes() const override;

private:
  dart::dynamics::BodyNodePtr mBodyNode;
  DifferentaiblePtr mInnerConstraint;
};


} // constraint
} // aikido

#endif