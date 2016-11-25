#include "PolynomialConstraint.hpp"
#include <aikido/util/RNG.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::statespace::Rn;

TEST(PolynomialConstraint, Constructor)
{
  PolynomialConstraint p(Eigen::Vector3d(1,2,3));
  EXPECT_THROW(PolynomialConstraint(Eigen::Vector3d(1,2,0)),
               std::invalid_argument);
}

TEST(PolynomialConstraint, GetValue)
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

}


TEST(PolynomialConstraint, GetJacobian)
{
  PolynomialConstraint p(Eigen::Vector3d(1,2,3));

  Eigen::VectorXd v(1);
  v(0) = -2;

  Rn rvss(1);
  auto s1 = rvss.createState();
  s1.setValue(v);

  Eigen::MatrixXd jac;
  p.getJacobian(s1, jac);
  EXPECT_DOUBLE_EQ(-10, jac(0,0));

}
