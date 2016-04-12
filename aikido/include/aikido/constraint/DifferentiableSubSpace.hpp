#ifndef AIKIDO_CONSTRAINT_DIFFERENTIABLESUBSPACE_H
#define AIKIDO_CONSTRAINT_DIFFERENTIABLESUBSPACE_H
#include "../statespace/CompoundStateSpace.hpp"
#include "Differentiable.hpp"

namespace aikido {
namespace constraint{

class DifferentiableSubSpace : public constraint::Differentiable
{
public:
  /// Apply _constraint to the i-th subspace of _stateSpace.
  DifferentiableSubSpace(
    std::shared_ptr<statespace::CompoundStateSpace> _stateSpace,
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

private:
  std::shared_ptr<statespace::CompoundStateSpace> mStateSpace;
  DifferentiablePtr mConstraint;
  size_t mIndex;
};

} // constraint
} // aikido

#endif // AIKIDO_CONSTRAINT_DIFFERENTIABLESUBSPACE_H
