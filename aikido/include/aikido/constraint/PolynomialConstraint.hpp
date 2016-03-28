#ifndef AIKIDO_CONSTRAINT_POLYNOMIALCONSTRAINT_H
#define AIKIDO_CONSTRAINT_POLYNOMIALCONSTRAINT_H

#include "Differentiable.hpp"

namespace aikido {
namespace constraint{

class PolynomialConstraint: public Differentiable
{
public:
  /// a0 + a1*x + a2*x^2 + ... + aN*x^N = 0.
  /// Last element (aN) should be non-zero.
  PolynomialConstraint(Eigen::VectorXd _coeffs);

  size_t getConstraintDimension() const override;

  Eigen::VectorXd getValue(const state::StatePtr& _s) const override;

  state::JacobianPtr getJacobian(const state::StatePtr& _s) const override;

  std::vector<ConstraintType> getConstraintTypes() const override;

private:
  Eigen::VectorXd mCoeffs;

};

} // constraint
} // aikido

#endif
