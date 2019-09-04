#ifndef AIKIDO_CONSTRAINT_DIFFERENTIABLEINTERSECTION_HPP_
#define AIKIDO_CONSTRAINT_DIFFERENTIABLEINTERSECTION_HPP_

#include <memory>

#include <Eigen/Dense>

#include "aikido/constraint/Differentiable.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace constraint {

/// Contains n constraints that take the same statespace.
/// getValue and getJacobian returns stacked vector and matrix.
class DifferentiableIntersection : public Differentiable
{
public:
  DifferentiableIntersection(
      std::vector<DifferentiablePtr> _constraints,
      statespace::ConstStateSpacePtr _stateSpace);

  // Documentation inherited.
  std::size_t getConstraintDimension() const override;

  // Documentation inherited.
  void getValue(const statespace::StateSpace::State* _s, Eigen::VectorXd& _out)
      const override;

  // Documentation inherited.
  void getJacobian(
      const statespace::StateSpace::State* _s,
      Eigen::MatrixXd& _out) const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  void getValueAndJacobian(
      const statespace::StateSpace::State* _s,
      Eigen::VectorXd& _val,
      Eigen::MatrixXd& _jac) const override;

private:
  std::vector<DifferentiablePtr> mConstraints;
  statespace::ConstStateSpacePtr mStateSpace;
};

} // namespace constraint
} // namespace aikido

#endif
