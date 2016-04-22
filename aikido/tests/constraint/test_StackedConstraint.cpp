#include "PolynomialConstraint.hpp"
#include <aikido/constraint/StackedConstraint.hpp>
#include <aikido/constraint/TSR.hpp>

#include <aikido/statespace/Rn.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <memory>

using aikido::constraint::StackedConstraint;
using aikido::constraint::TSR;
using aikido::constraint::DifferentiablePtr;

using aikido::statespace::Rn;
using aikido::statespace::StateSpace;
using aikido::statespace::StateSpacePtr;


TEST(StackedConstraint, InvalidConstructor)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<Rn> rvss(new Rn(1));

  // empty constraints
  EXPECT_THROW(StackedConstraint(constraints, rvss), std::invalid_argument);

  // null statespace
  StateSpacePtr space;
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector3d(1, 2, 3), rvss));
  EXPECT_THROW(StackedConstraint(constraints, space), std::invalid_argument);

  // constraints have different space
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector3d(1, 2, 3), rvss));
  constraints.push_back(std::make_shared<TSR>());
  EXPECT_THROW(StackedConstraint(constraints, rvss), std::invalid_argument);
}

TEST(StackedConstraint, getValue)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<Rn> rvss(new Rn(1));

  // constraint1: 1 + 2x + 3x^2
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector3d(1, 2, 3), rvss));

  // constraint2: 4 + 5x
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector2d(4, 5), rvss));

  Eigen::VectorXd v(1);
  v(0) = -2;

  auto s1 = rvss->createState();
  s1.setValue(v);

  StackedConstraint stacked(constraints, rvss);

  Eigen::Vector2d expected(9, -6);
  EXPECT_TRUE(stacked.getValue(s1).isApprox(expected));
}


TEST(StackedConstraint, getJacobian)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<Rn> rvss(new Rn(1));

  // constraint1: 1 + 2x + 3x^2
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector3d(1, 2, 3), rvss));

  // constraint2: 4 + 5x
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector2d(4, 5), rvss));

  Eigen::VectorXd v(1);
  v(0) = -2;

  auto s1 = rvss->createState();
  s1.setValue(v);

  StackedConstraint stacked(constraints, rvss);
  Eigen::MatrixXd expected(2, 1);
  expected(0, 0) = -10;
  expected(1, 0) = 5;

  EXPECT_TRUE(stacked.getJacobian(s1).isApprox(expected));
}

