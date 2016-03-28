#include <aikido/constraint/PolynomialConstraint.hpp>
#include <aikido/state/State.hpp>
#include <aikido/state/Jacobian.hpp>

#include <aikido/util/RNG.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::constraint::PolynomialConstraint;
using namespace aikido::state; 

TEST(PolynomialConstraint, Constructor)
{

  PolynomialConstraint p(Eigen::Vector3d(1,2,3));
  EXPECT_THROW(PolynomialConstraint(Eigen::Vector3d(1,2,0)),
               std::invalid_argument);
}

TEST(PolynomialConstraint, GetValue)
{

  PolynomialConstraint p(Eigen::Vector3d(1,2,3));

  Eigen::VectorXd value = p.getValue(std::make_shared<RealVectorState>(0));

  EXPECT_DOUBLE_EQ(1, value(0));
  EXPECT_DOUBLE_EQ(p.getValue(std::make_shared<RealVectorState>(-2))(0), 9);

}


TEST(PolynomialConstraint, Jacobian)
{

  PolynomialConstraint p(Eigen::Vector3d(1,2,3));

  JacobianPtr jac = p.getJacobian(std::make_shared<RealVectorState>(0));
  RealVectorJacobianPtr rvJac = std::dynamic_pointer_cast<RealVectorJacobian>(jac);

  EXPECT_DOUBLE_EQ(2, rvJac->mJacobian(0));


  jac = p.getJacobian(std::make_shared<RealVectorState>(-2));
  rvJac =  std::dynamic_pointer_cast<RealVectorJacobian>(jac);

  EXPECT_DOUBLE_EQ(-10, rvJac->mJacobian(0));

}