#ifndef AIKIDO_TESTS_CONSTRAINT_POLYNOMIALCONSTRAINT_HPP_
#define AIKIDO_TESTS_CONSTRAINT_POLYNOMIALCONSTRAINT_HPP_

#include "Differentiable.hpp"
#include <aikido/statespace/RealVectorStateSpace.hpp>


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

  /// Documentation inherited.
  std::vector<ConstraintType> getConstraintTypes() const override;

  /// Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getValueAndJacobian(
    const statespace::StateSpace::State* _s) const override;

private:
  Eigen::VectorXd mCoeffs;
  statespace::RealVectorStateSpace mStateSpace;

};


#endif
