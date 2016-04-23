#ifndef AIKIDO_CONSTRAINT_DIFFERENTIABLE_HPP_
#define AIKIDO_CONSTRAINT_DIFFERENTIABLE_HPP_

#include "../statespace/StateSpace.hpp"
#include <Eigen/Dense>
#include <memory>

namespace aikido {
namespace constraint {

/// Enum for classifying constraints used in Differentiable.
/// Equality constraint is satisfied by f(x) = 0.
/// Inequality constraint is satisfied by f(x) <= 0. 
enum class ConstraintType {EQUALITY, INEQUALITY};

/// A differentiable constraint.
/// Contains n constraints that can be evaluated in real-value. 
class Differentiable
{
public:
  virtual ~Differentiable() = default;

  /// Gets the StateSpace that this constraint operates on.
  virtual statespace::StateSpacePtr getStateSpace() const = 0;

  /// Returns a vector of constraints' types, i-th element correspoinding to 
  /// the type of i-th constraint.
  virtual std::vector<ConstraintType> getConstraintTypes() const = 0;

  /// Size of constraints
  virtual size_t getConstraintDimension() const = 0;

  /// Value of constraints at _s.
  /// Should be 0 to satisfy equality constraints.
  /// Should be <=0 to satisfy inequality constraints.
  virtual Eigen::VectorXd getValue(
    const statespace::StateSpace::State* _s) const = 0;

  /// Jacobian of constraints at _s,
  /// expressed in the frame each state space is expressed in).
  /// For SO3 StateSpace: m x 3 
  ///     SO2           : m x 1
  ///     SE2           : m x 3
  ///     SE3           : m x 6 
  ///     RealVector(n) : m x n
  ///     Compound      : m x k (k = sum of all state jacobian cols)
  virtual Eigen::MatrixXd getJacobian(
    const statespace::StateSpace::State* _s) const = 0;

  /// Returns (Value, Jacobian).
  virtual std::pair<Eigen::VectorXd, Eigen::MatrixXd> getValueAndJacobian(
    const statespace::StateSpace::State* _s) const;
};

using DifferentiablePtr = std::shared_ptr<Differentiable>;

} // constraint
} // aikido

#endif
