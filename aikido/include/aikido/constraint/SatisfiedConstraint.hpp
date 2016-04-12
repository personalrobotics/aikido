#ifndef AIKIDO_CONSTRAINT_SATISFIEDCONSTRAINT_H_
#define AIKIDO_CONSTRAINT_SATISFIEDCONSTRAINT_H_
#include "Differentiable.hpp"
#include "Projectable.hpp"
#include "Sampleable.hpp"
#include "TestableConstraint.hpp"

namespace aikido {
namespace constraint {

class SatisfiedConstraint 
  : public constraint::Differentiable
  , public constraint::Projectable
  , public constraint::TestableConstraint
{
public:
  explicit SatisfiedConstraint(statespace::StateSpacePtr _space);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  size_t getConstraintDimension() const override;

  // Documentation inherited.
  std::vector<constraint::ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  bool isSatisfied(const statespace::StateSpace::State* state) const override;

  // Documentation inherited.
  bool project(
    const statespace::StateSpace::State* _s,
    statespace::StateSpace::State* _out) const override;

  // Documentation inherited.
  Eigen::VectorXd getValue(
    const statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  Eigen::MatrixXd getJacobian(
    const statespace::StateSpace::State* _s) const override;

private:
  statespace::StateSpacePtr mStateSpace;
};

} // namespace constraint
} // namespace aikido

#endif // ifndef AIKIDO_CONSTRAINT_SATISFIEDCONSTRAINT_H_
