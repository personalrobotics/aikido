#ifndef AIKIDO_CONSTRAINT_DIFFERENTIABLE_HPP_
#define AIKIDO_CONSTRAINT_DIFFERENTIABLE_HPP_

#include <memory>
#include <Eigen/Dense>
#include "aikido/statespace/smart_pointer.hpp"
#include "../statespace/StateSpace.hpp"

namespace aikido {
namespace constraint {

/// Enum for classifying constraints used in Differentiable.
/// Equality constraint is satisfied by f(x) = 0.
/// Inequality constraint is satisfied by f(x) <= 0.
enum class ConstraintType
{
  EQUALITY,
  INEQUALITY
};

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
  virtual std::size_t getConstraintDimension() const = 0;

  /// Get the value of constraints at _s.
  /// Should be 0 to satisfy equality constraints.
  /// Should be <=0 to satisfy inequality constraints.
  /// \param _s State to be evaluated at.
  /// \param[out] _out Vector to store the value. Length should match the number
  ///        of constraints.
  virtual void getValue(
      const statespace::StateSpace::State* _s, Eigen::VectorXd& _out) const = 0;

  /// Get the jacobian of constraints evaluated at _s,
  /// expressed in the frame each state space is expressed in).
  /// \param _s State to be evaluated at.
  /// \param[out] _out Jacobian matrix. The dimension should be the following:
  ///     SO3 StateSpace  : m x 3
  ///     SO2             : m x 1
  ///     SE2             : m x 3
  ///     SE3             : m x 6
  ///     Rn              : m x n
  ///     CartesianProduct: m x k (k = sum of all state jacobian cols)
  /// If Matrix of incorrect dimension is given, false will be returned.
  virtual void getJacobian(
      const statespace::StateSpace::State* _s, Eigen::MatrixXd& _out) const = 0;

  /// Get both Value and Jacobian.
  /// \param _s State to be evaluated.
  /// \param[out] _val Value of constraints.
  /// \param[out] _jac Jacoiban of constraints.
  virtual void getValueAndJacobian(
      const statespace::StateSpace::State* _s,
      Eigen::VectorXd& _val,
      Eigen::MatrixXd& _jac) const;
};

} // namespace constraint
} // namespace aikido

#endif
