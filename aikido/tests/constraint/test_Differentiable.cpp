#include <gtest/gtest.h>
#include "../eigen_tests.hpp"

#include <aikido/constraint/Differentiable.hpp>
#include "PolynomialConstraint.hpp"

using aikido::constraint::DifferentiablePtr;
using aikido::statespace::Rn;

TEST(Differentiable, GetValueAndJacobianDefault)
{

  PolynomialConstraint p(Eigen::Vector3d(1,2,3));

  Eigen::VectorXd v(1);
  v(0) = -2;

  Rn rvss(1);
  auto s1 = rvss.createState();
  s1.setValue(v);

  Eigen::VectorXd value;
  p.getValue(s1, value);
  EXPECT_DOUBLE_EQ(value(0), 9);

  Eigen::MatrixXd jac;
  p.getJacobian(s1, jac);
  EXPECT_DOUBLE_EQ(-10, jac(0,0));

  std::pair<Eigen::VectorXd, Eigen::MatrixXd> valueJacPair;
  p.getValueAndJacobian(s1, valueJacPair);

  EXPECT_TRUE(value.isApprox(valueJacPair.first));
  EXPECT_TRUE(jac.isApprox(valueJacPair.second));
}


