#include "PolynomialConstraint.hpp"
#include <aikido/constraint/StackedConstraint.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <memory>

using aikido::constraint::PolynomialConstraint;
using aikido::constraint::StackedConstraint;
using aikido::constraint::DifferentiablePtr;

using aikido::statespace::RealVectorStateSpace;
using aikido::statespace::StateSpace;

TEST(StackedConstraint, getValue)
{
  std::vector<DifferentiablePtr> constraints;

  // constraint1: 1 + 2x + 3x^2
  DifferentiablePtr p1 = std::make_shared<PolynomialConstraint>(Eigen::Vector3d(1, 2, 3));
  constraints.push_back(p1);

  // constraint2: 4 + 5x
  constraints.push_back(std::make_shared<PolynomialConstraint>(Eigen::Vector2d(4, 5)));

  Eigen::VectorXd v(1);
  v(0) = -2;

  RealVectorStateSpace rvss(1);
  auto s1 = rvss.createState();
  s1.setValue(v);

  StackedConstraint stacked(constraints, std::make_shared<RealVectorStateSpace>(rvss));

  Eigen::Vector2d expected(9, -6);
  EXPECT_TRUE(stacked.getValue(s1).isApprox(expected));
}


TEST(StackedConstraint, getJacobian)
{
  std::vector<DifferentiablePtr> constraints;

  // constraint1: 1 + 2x + 3x^2
  DifferentiablePtr p1 = std::make_shared<PolynomialConstraint>(Eigen::Vector3d(1, 2, 3));
  constraints.push_back(p1);

  // constraint2: 4 + 5x
  constraints.push_back(std::make_shared<PolynomialConstraint>(Eigen::Vector2d(4, 5)));

  Eigen::VectorXd v(1);
  v(0) = -2;

  RealVectorStateSpace rvss(1);
  auto s1 = rvss.createState();
  s1.setValue(v);

  StackedConstraint stacked(constraints, std::make_shared<RealVectorStateSpace>(rvss));

  Eigen::MatrixXd expected(2, 1);
  expected(0, 0) = -10;
  expected(1, 0) = 5;

  EXPECT_TRUE(stacked.getJacobian(s1).isApprox(expected));
}