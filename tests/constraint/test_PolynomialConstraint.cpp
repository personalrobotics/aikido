#include "PolynomialConstraint.hpp"
#include <aikido/common/RNG.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::statespace::R1;

TEST(PolynomialConstraint, Constructor)
{
  PolynomialConstraint<1> p(Eigen::Vector3d(1,2,3));
  EXPECT_THROW(PolynomialConstraint<1>(Eigen::Vector3d(1,2,0)),
               std::invalid_argument);
}

TEST(PolynomialConstraint, GetValue)
{
  PolynomialConstraint<1> p(Eigen::Vector3d(1,2,3));

  Eigen::VectorXd v(1);
  v(0) = -2;

  R1 rvss;
  auto s1 = rvss.createState();
  s1.setValue(v);

  Eigen::VectorXd value;
  p.getValue(s1, value);

  EXPECT_DOUBLE_EQ(value(0), 9);

}


TEST(PolynomialConstraint, GetJacobian)
{
  PolynomialConstraint<1> p(Eigen::Vector3d(1,2,3));

  Eigen::VectorXd v(1);
  v(0) = -2;

  R1 rvss;
  auto s1 = rvss.createState();
  s1.setValue(v);

  Eigen::MatrixXd jac;
  p.getJacobian(s1, jac);
  EXPECT_DOUBLE_EQ(-10, jac(0,0));

}
