#ifndef AIKIDO_TESTS_CONSTRAINT_POLYNOMIALCONSTRAINT_HPP_
#define AIKIDO_TESTS_CONSTRAINT_POLYNOMIALCONSTRAINT_HPP_

#include <aikido/constraint/Differentiable.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>


class PolynomialConstraint: public aikido::constraint::Differentiable
{
public:
  /// a0 + a1*x + a2*x^2 + ... + aN*x^N = 0.
  /// Last element (aN) should be non-zero.
  explicit PolynomialConstraint(Eigen::VectorXd _coeffs);

  PolynomialConstraint(Eigen::VectorXd _coeffs, 
    std::shared_ptr<aikido::statespace::RealVectorStateSpace> _space);

  // Documentation inherited.
  size_t getConstraintDimension() const override;

  // Documentation inherited.
  Eigen::VectorXd getValue(
      const aikido::statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  Eigen::MatrixXd getJacobian(
      const aikido::statespace::StateSpace::State* _s) const override;

  // Documentation inherited.
  std::vector<aikido::constraint::ConstraintType> getConstraintTypes()
      const override;

  // Documentation inherited.
  aikido::statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> getValueAndJacobian(
      const aikido::statespace::StateSpace::State* _s) const override;

private:
  Eigen::VectorXd mCoeffs;
  std::shared_ptr<aikido::statespace::RealVectorStateSpace> mStateSpace;
};


#endif
