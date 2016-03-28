#ifndef AIKIDO_CONSTRAINT_DIFFERENTIABLE_H
#define AIKIDO_CONSTRAINT_DIFFERENTIABLE_H

#include "../state/Jacobian.hpp"
#include "../state/State.hpp"
#include <Eigen/Dense>
#include <memory>

namespace aikido {
namespace constraint{

enum class ConstraintType {EQ, INEQ};

/// A differentiable constraint.
class Differentiable
{
public:

  /// Size of constraints
  virtual size_t getConstraintDimension() const = 0;

  /// Value of constraints at _s.
  /// Should be 0 to satisfy equality constraints.
  /// Should be <=0 to satisfy inequality constraints.
  virtual Eigen::VectorXd getValue(const state::StatePtr& _s) const = 0;

  /// Jacobian of constraints at _s.
  /// For SO3 state, returns mx3 matrix.
  /// For SO2 state, returns mx1 matrix.
  /// For vector<n> state, returns mxn matrix.
  /// For compound state with k components, returns k matrices 
  /// where ith value correcsponds to Jacobian at ith component.
  virtual state::JacobianPtr getJacobian(const state::StatePtr& _s) const = 0;

  /// Returns a vector containing each constraint's type.
  virtual std::vector<ConstraintType> getConstraintTypes() const = 0;
};

using DifferentiablePtr = std::shared_ptr<Differentiable>;

} // constraint
} // aikido

#endif
