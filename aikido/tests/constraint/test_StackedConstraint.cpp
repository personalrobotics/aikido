#include "PolynomialConstraint.hpp"
#include <aikido/constraint/StackedConstraint.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <memory>

using aikido::constraint::StackedConstraint;
using aikido::constraint::TSR;
using aikido::constraint::DifferentiablePtr;

using aikido::statespace::RealVectorStateSpace;
using aikido::statespace::StateSpace;
using aikido::statespace::StateSpacePtr;

TEST(StackedConstraint, InvalidConstructor)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<RealVectorStateSpace> rvss(new RealVectorStateSpace(1));

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

TEST(StackedConstraint, GetValue)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<RealVectorStateSpace> rvss(new RealVectorStateSpace(1));

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


TEST(StackedConstraint, GetJacobian)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<RealVectorStateSpace> rvss(new RealVectorStateSpace(1));

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

/// Compare returned values with getValue and getJacobian
TEST(StackedConstraint, GetValueAndJacobianMatchValueAndJacobian)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<RealVectorStateSpace> rvss(new RealVectorStateSpace(1));

  // constraint1: 1 + 2x + 3x^2
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector3d(1, 2, 3), rvss));

  // constraint2: 4 + 5x
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector2d(4, 5), rvss));

  auto s1 = rvss->createState();
  Eigen::VectorXd v(1);
  v(0) = -2;
  s1.setValue(v);

  StackedConstraint stacked(constraints, rvss);
  auto valueAndJacobian = stacked.getValueAndJacobian(s1);
  auto value = stacked.getValue(s1);
  auto jacobian = stacked.getJacobian(s1);

  EXPECT_TRUE(valueAndJacobian.first.isApprox(value));
  EXPECT_TRUE(valueAndJacobian.second.isApprox(jacobian));
}

TEST(StackedConstraint, GetConstraintTypes)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<RealVectorStateSpace> rvss(new RealVectorStateSpace(1));

  // constraint1: 1 + 2x + 3x^2
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector3d(1, 2, 3), rvss));

  // constraint2: 4 + 5x
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector2d(4, 5), rvss));

  StackedConstraint stacked(constraints, rvss);
  auto constraintTypes = stacked.getConstraintTypes();

  // Check the size of ConstraintTypes
  EXPECT_EQ(constraintTypes.size(), 2);

  // Check the types
  for (auto cType: constraintTypes)
  {
    EXPECT_EQ(cType, aikido::constraint::ConstraintType::EQUALITY);
  }
}

TEST(StackedConstraint, GetStateSpace)
{
  std::vector<DifferentiablePtr> constraints;
  std::shared_ptr<RealVectorStateSpace> rvss(new RealVectorStateSpace(1));

  // constraint1: 1 + 2x + 3x^2
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector3d(1, 2, 3), rvss));

  // constraint2: 4 + 5x
  constraints.push_back(std::make_shared<PolynomialConstraint>(
    Eigen::Vector2d(4, 5), rvss));

  StackedConstraint stacked(constraints, rvss);
  auto space = stacked.getStateSpace();

  EXPECT_EQ(space, rvss);
}


