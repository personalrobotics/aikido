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

  Eigen::VectorXd valExpected;
  p.getValue(s1, valExpected);
  EXPECT_DOUBLE_EQ(valExpected(0), 9);

  Eigen::MatrixXd jacExpected;
  p.getJacobian(s1, jacExpected);
  EXPECT_DOUBLE_EQ(-10, jacExpected(0,0));

  Eigen::VectorXd val;
  Eigen::MatrixXd jac;

  p.getValueAndJacobian(s1, val, jac);

  EXPECT_TRUE(valExpected.isApprox(val));
  EXPECT_TRUE(jacExpected.isApprox(jac));
}


