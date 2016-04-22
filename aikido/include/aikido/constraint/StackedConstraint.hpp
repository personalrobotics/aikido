#ifndef AIKIDO_CONSTRAINT_STACKEDCONSTRAINT_HPP_
#define AIKIDO_CONSTRAINT_STACKEDCONSTRAINT_HPP_

#include "../statespace/StateSpace.hpp"
#include "Differentiable.hpp"
#include <Eigen/Dense>
#include <memory>

namespace aikido {
namespace constraint{

/// Contains n constraints that take the same statespace. 
/// getValue and getJacobian returns stacked vector and matrix.
class StackedConstraint : public Differentiable
{
public:
  StackedConstraint(
    std::vector<DifferentiablePtr> _constraints,
    statespace::StateSpacePtr _stateSpace);

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

  // Documentation inherited. 
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getValueAndJacobian(
    const statespace::StateSpace::State* _s) const override;

private:
  std::vector<DifferentiablePtr> mConstraints;
  std::shared_ptr<aikido::statespace::StateSpace> mStateSpace;

};


} // constraint
} // aikido

#endif
