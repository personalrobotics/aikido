#ifndef AIKIDO_CONSTRAINT_DIFFERENTIABLESUBSPACE_HPP_
#define AIKIDO_CONSTRAINT_DIFFERENTIABLESUBSPACE_HPP_
#include "../statespace/CartesianProduct.hpp"
#include "Differentiable.hpp"

namespace aikido {
namespace constraint{

/// A differentiable constraint applied only on a subspace of
/// a CompoundState.
class DifferentiableSubSpace : public Differentiable
{
public:
  /// Apply _constraint to the i-th subspace of _stateSpace.
  /// \param _stateSpace CartesianProduct.
  /// \param _constraint Constraint being applied.
  /// \param _index Subspace of _stateSpace to apply _constraint.
  DifferentiableSubSpace(
    std::shared_ptr<statespace::CartesianProduct> _stateSpace,
    DifferentiablePtr _constraint, size_t _index);

  virtual ~DifferentiableSubSpace() = default;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  size_t getConstraintDimension() const override;

  // Documentation inherited.
  Eigen::VectorXd getValue(
    const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  Eigen::MatrixXd getJacobian(
    const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getValueAndJacobian(
    const statespace::StateSpace::State* _s) const override;

private:
  std::shared_ptr<statespace::CartesianProduct> mStateSpace;
  DifferentiablePtr mConstraint;
  size_t mIndex;
};

} // constraint
} // aikido

#endif // AIKIDO_CONSTRAINT_DIFFERENTIABLESUBSPACE_HPP_
