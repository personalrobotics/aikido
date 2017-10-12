#ifndef AIKIDO_CONSTRAINT_DIFFERENTIABLESUBSPACE_HPP_
#define AIKIDO_CONSTRAINT_DIFFERENTIABLESUBSPACE_HPP_

#include "../statespace/CartesianProduct.hpp"
#include "Differentiable.hpp"

namespace aikido {
namespace constraint {

/// A differentiable constraint applied only on a subspace of
/// a CompoundState.
class DifferentiableSubspace : public Differentiable
{
public:
  /// Apply _constraint to the i-th subspace of _stateSpace.
  /// \param _stateSpace CartesianProduct.
  /// \param _constraint Constraint being applied.
  /// \param _index Subspace of _stateSpace to apply _constraint.
  DifferentiableSubspace(
      std::shared_ptr<statespace::CartesianProduct> _stateSpace,
      DifferentiablePtr _constraint,
      std::size_t _index);

  virtual ~DifferentiableSubspace() = default;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

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
  void getValueAndJacobian(
      const statespace::StateSpace::State* _s,
      Eigen::VectorXd& _val,
      Eigen::MatrixXd& _jac) const override;

private:
  std::shared_ptr<statespace::CartesianProduct> mStateSpace;
  DifferentiablePtr mConstraint;
  std::size_t mIndex;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DIFFERENTIABLESUBSPACE_HPP_
