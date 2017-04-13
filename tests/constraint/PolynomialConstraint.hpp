#ifndef AIKIDO_TESTS_CONSTRAINT_POLYNOMIALCONSTRAINT_HPP_
#define AIKIDO_TESTS_CONSTRAINT_POLYNOMIALCONSTRAINT_HPP_

#include <aikido/constraint/Differentiable.hpp>
#include <aikido/statespace/Rn.hpp>

template <int N = 1>
class PolynomialConstraint: public aikido::constraint::Differentiable
{
public:
  /// a0 + a1*x + a2*x^2 + ... + aN*x^N = 0.
  /// Last element (aN) should be non-zero.
  PolynomialConstraint(const Eigen::VectorXd& _coeffs,
    std::shared_ptr<aikido::statespace::R<N>> _space
        = std::make_shared<aikido::statespace::R<N>>());

  // Documentation inherited.
  size_t getConstraintDimension() const override;

  // Documentation inherited.
  void getValue(
      const aikido::statespace::StateSpace::State* _s,
      Eigen::VectorXd& _out) const override;

  // Documentation inherited.
  void getJacobian(
      const aikido::statespace::StateSpace::State* _s,
      Eigen::MatrixXd& _out) const override;

  /// These are all equality constriants.
  std::vector<aikido::constraint::ConstraintType> getConstraintTypes()
      const override;

  // Documentation inherited.
  aikido::statespace::StateSpacePtr getStateSpace() const override;

private:
  Eigen::VectorXd mCoeffs;
  std::shared_ptr<aikido::statespace::R<N>> mStateSpace;
};

#include "PolynomialConstraint-impl.hpp"

#endif
