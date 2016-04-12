#ifndef AIKIDO_CONSTRAINT_STACKEDCONSTRAINT_H
#define AIKIDO_CONSTRAINT_STACKEDCONSTRAINT_H

#include "../statespace/StateSpace.hpp"
#include "Differentiable.hpp"
#include <Eigen/Dense>
#include <memory>

namespace aikido {
namespace constraint{

/// A differentiable constraint.
/// Contains n constraints that can be evaluated in real-value. 
class StackedConstraint : public Differentiable
{
public:
  StackedConstraint(const std::vector<DifferentiablePtr>& _constraints);

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
  std::vector<DifferentiablePtr> mConstraints;

};


} // constraint
} // aikido

#endif
