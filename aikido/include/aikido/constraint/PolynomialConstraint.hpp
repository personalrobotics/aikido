#ifndef AIKIDO_CONSTRAINT_POLYNOMIALCONSTRAINT_H
#define AIKIDO_CONSTRAINT_POLYNOMIALCONSTRAINT_H

#include "Differentiable.hpp"
#include <aikido/statespace/RealVectorStateSpace.hpp>

namespace aikido {
namespace constraint{

class PolynomialConstraint: public Differentiable
{
public:
  /// a0 + a1*x + a2*x^2 + ... + aN*x^N = 0.
  /// Last element (aN) should be non-zero.
  PolynomialConstraint(Eigen::VectorXd _coeffs);

  /// Documentation inherited.
  size_t getConstraintDimension() const override;

  /// Documentation inherited.
  Eigen::VectorXd getValue(
    const statespace::StateSpace::State* _s) const override;

 	/// Documentation inherited.
  Eigen::MatrixXd getJacobian(
    const statespace::StateSpace::State* _s) const override;

  /// Returns a vector of constraints' types.
  std::vector<ConstraintType> getConstraintTypes() const override;

  /// Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

private:
  Eigen::VectorXd mCoeffs;
  statespace::RealVectorStateSpace mStateSpace;

};

} // constraint
} // aikido

#endif
