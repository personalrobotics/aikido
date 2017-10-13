#include <memory>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <aikido/constraint/DifferentiableIntersection.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/statespace/Rn.hpp>
#include "PolynomialConstraint.hpp"

using aikido::constraint::DifferentiableIntersection;
using aikido::constraint::TSR;
using aikido::constraint::DifferentiablePtr;

using aikido::statespace::R1;
using aikido::statespace::StateSpace;
using aikido::statespace::StateSpacePtr;

TEST(DifferentiableIntersection, InvalidConstructor)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<R1> rvss(new R1());

  // empty constraints
  EXPECT_THROW(
      DifferentiableIntersection(constraints, rvss), std::invalid_argument);

  // null statespace
  StateSpacePtr space;
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(
          Eigen::Vector3d(1, 2, 3), rvss));
  EXPECT_THROW(
      DifferentiableIntersection(constraints, space), std::invalid_argument);

  // constraints have different space
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(
          Eigen::Vector3d(1, 2, 3), rvss));
  constraints.push_back(dart::common::make_aligned_shared<TSR>());
  EXPECT_THROW(
      DifferentiableIntersection(constraints, rvss), std::invalid_argument);
}

TEST(DifferentiableIntersection, getValue)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<R1> rvss(new R1());

  // constraint1: 1 + 2x + 3x^2
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(
          Eigen::Vector3d(1, 2, 3), rvss));

  // constraint2: 4 + 5x
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(Eigen::Vector2d(4, 5), rvss));

  Eigen::VectorXd v(1);
  v(0) = -2;

  auto s1 = rvss->createState();
  s1.setValue(v);

  DifferentiableIntersection stacked(constraints, rvss);

  Eigen::Vector2d expected(9, -6);
  Eigen::VectorXd out;
  stacked.getValue(s1, out);

  EXPECT_TRUE(out.isApprox(expected));
}

TEST(DifferentiableIntersection, getJacobian)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<R1> rvss(new R1());

  // constraint1: 1 + 2x + 3x^2
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(
          Eigen::Vector3d(1, 2, 3), rvss));

  // constraint2: 4 + 5x
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(Eigen::Vector2d(4, 5), rvss));

  Eigen::VectorXd v(1);
  v(0) = -2;

  auto s1 = rvss->createState();
  s1.setValue(v);

  DifferentiableIntersection stacked(constraints, rvss);
  Eigen::MatrixXd expected(2, 1);
  expected(0, 0) = -10;
  expected(1, 0) = 5;

  Eigen::MatrixXd out;
  stacked.getJacobian(s1, out);

  EXPECT_TRUE(out.isApprox(expected));
}

/// Compare returned values with getValue and getJacobian
TEST(DifferentiableIntersection, GetValueAndJacobianMatchValueAndJacobian)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<R1> rvss(new R1());

  // constraint1: 1 + 2x + 3x^2
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(
          Eigen::Vector3d(1, 2, 3), rvss));

  // constraint2: 4 + 5x
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(Eigen::Vector2d(4, 5), rvss));

  auto s1 = rvss->createState();
  Eigen::VectorXd v(1);
  v(0) = -2;
  s1.setValue(v);

  DifferentiableIntersection stacked(constraints, rvss);
  Eigen::VectorXd val;
  Eigen::MatrixXd jac;
  stacked.getValueAndJacobian(s1, val, jac);

  Eigen::VectorXd value;
  Eigen::MatrixXd jacobian;

  stacked.getValue(s1, value);
  stacked.getJacobian(s1, jacobian);

  EXPECT_TRUE(val.isApprox(value));
  EXPECT_TRUE(jac.isApprox(jacobian));
}

TEST(DifferentiableIntersection, GetConstraintTypes)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<R1> rvss(new R1());

  // constraint1: 1 + 2x + 3x^2
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(
          Eigen::Vector3d(1, 2, 3), rvss));

  // constraint2: 4 + 5x
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(Eigen::Vector2d(4, 5), rvss));

  DifferentiableIntersection stacked(constraints, rvss);
  auto constraintTypes = stacked.getConstraintTypes();

  // Check the size of ConstraintTypes
  EXPECT_EQ(constraintTypes.size(), 2);

  // Check the types
  for (auto cType : constraintTypes)
  {
    EXPECT_EQ(cType, aikido::constraint::ConstraintType::EQUALITY);
  }
}

TEST(DifferentiableIntersection, GetStateSpace)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<R1> rvss(new R1());

  // constraint1: 1 + 2x + 3x^2
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(
          Eigen::Vector3d(1, 2, 3), rvss));

  // constraint2: 4 + 5x
  constraints.push_back(
      std::make_shared<PolynomialConstraint<1>>(Eigen::Vector2d(4, 5), rvss));

  DifferentiableIntersection stacked(constraints, rvss);
  auto space = stacked.getStateSpace();

  EXPECT_EQ(space, rvss);
}
