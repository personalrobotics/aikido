#include <gtest/gtest.h>
#include <aikido/constraint/DifferentiableSubspace.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include "../eigen_tests.hpp"
#include "PolynomialConstraint.hpp"

using aikido::constraint::DifferentiableSubspace;
using aikido::constraint::Satisfied;
using aikido::statespace::CartesianProduct;
using aikido::statespace::SO2;
using aikido::statespace::R1;
using aikido::statespace::R3;

class DifferentiableSubspaceTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    auto so2 = std::make_shared<SO2>();
    constraint
        = std::make_shared<PolynomialConstraint<1>>(Eigen::Vector3d(-1, 0, 1));
    auto rv = constraint->getStateSpace();

    cs = std::make_shared<CartesianProduct>(
        std::vector<aikido::statespace::StateSpacePtr>({so2, rv}));
    ds = std::make_shared<DifferentiableSubspace>(cs, constraint, 1);
  }

  std::shared_ptr<PolynomialConstraint<1>> constraint;
  std::shared_ptr<CartesianProduct> cs;
  std::shared_ptr<DifferentiableSubspace> ds;
};

TEST_F(DifferentiableSubspaceTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(
      DifferentiableSubspace(nullptr, constraint, 1), std::invalid_argument);
}

TEST_F(DifferentiableSubspaceTest, ConstructorThrowsOnNullConstraint)
{
  auto so2 = std::make_shared<SO2>();
  auto rv = std::make_shared<R3>();
  auto constraint = std::make_shared<Satisfied>(rv);
  auto cs = std::make_shared<CartesianProduct>(
      std::vector<aikido::statespace::StateSpacePtr>({so2, rv}));

  EXPECT_THROW(DifferentiableSubspace(cs, nullptr, 1), std::invalid_argument);
}

TEST_F(DifferentiableSubspaceTest, ConstructorThrowsOnInvalidIndex)
{
  auto so2 = std::make_shared<SO2>();
  auto rv = std::make_shared<R3>();
  auto constraint = std::make_shared<Satisfied>(rv);
  auto cs = std::make_shared<CartesianProduct>(
      std::vector<aikido::statespace::StateSpacePtr>({so2, rv}));

  EXPECT_THROW(
      DifferentiableSubspace(cs, constraint, 2), std::invalid_argument);
}

TEST_F(DifferentiableSubspaceTest, ConstructorThrowsOnMismatchStateSpace)
{
  auto so2 = std::make_shared<SO2>();
  auto rv = std::make_shared<R3>();
  auto constraint = std::make_shared<Satisfied>(rv);
  auto cs = std::make_shared<CartesianProduct>(
      std::vector<aikido::statespace::StateSpacePtr>({so2, rv}));

  EXPECT_THROW(
      DifferentiableSubspace(cs, constraint, 0), std::invalid_argument);
}

TEST_F(DifferentiableSubspaceTest, StateSpace)
{
  EXPECT_EQ(cs, ds->getStateSpace());
}

TEST_F(DifferentiableSubspaceTest, ConstraintType)
{
  auto ctypes = ds->getConstraintTypes();
  EXPECT_EQ(1, ctypes.size());
  EXPECT_EQ(aikido::constraint::ConstraintType::EQUALITY, ctypes[0]);
}

TEST_F(DifferentiableSubspaceTest, ConstraintDimension)
{
  EXPECT_EQ(1, ds->getConstraintDimension());
}

TEST_F(DifferentiableSubspaceTest, ConstraintValue)
{
  auto st = cs->createState();
  auto subSpace = cs->getSubspace<R1>(1);
  auto subState = cs->getSubStateHandle<R1>(st, 1);

  subSpace->setValue(subState, aikido::tests::make_vector(2));

  Eigen::VectorXd expected = aikido::tests::make_vector(3);
  Eigen::VectorXd value;
  ds->getValue(st, value);

  EXPECT_TRUE(value.isApprox(expected));
}

TEST_F(DifferentiableSubspaceTest, ConstraintJacobian)
{
  auto st = cs->createState();
  auto subSpace = cs->getSubspace<R1>(1);
  auto subState = cs->getSubStateHandle<R1>(st, 1);

  subSpace->setValue(subState, aikido::tests::make_vector(2));

  Eigen::VectorXd expected = aikido::tests::make_vector(4);

  Eigen::MatrixXd jacobian;
  ds->getJacobian(st, jacobian);

  EXPECT_TRUE(jacobian.isApprox(expected));
}

TEST_F(DifferentiableSubspaceTest, ConstraintValueAndJacobian)
{
  auto st = cs->createState();
  auto subSpace = cs->getSubspace<R1>(1);
  auto subState = cs->getSubStateHandle<R1>(st, 1);

  subSpace->setValue(subState, aikido::tests::make_vector(2));

  Eigen::VectorXd expectedVal = aikido::tests::make_vector(3);
  Eigen::VectorXd expectedJac = aikido::tests::make_vector(4);

  Eigen::VectorXd val;
  Eigen::MatrixXd jac;
  ds->getValueAndJacobian(st, val, jac);
  EXPECT_TRUE(val.isApprox(expectedVal));
  EXPECT_TRUE(jac.isApprox(expectedJac));
}
