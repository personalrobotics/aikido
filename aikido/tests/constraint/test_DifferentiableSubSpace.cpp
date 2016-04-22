#include <gtest/gtest.h>
#include "PolynomialConstraint.hpp"
#include "../eigen_tests.hpp"
#include <aikido/constraint/DifferentiableSubSpace.hpp>
#include <aikido/constraint/SatisfiedConstraint.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>

using aikido::constraint::DifferentiableSubSpace;
using aikido::constraint::SatisfiedConstraint;
using aikido::statespace::CompoundStateSpace;
using aikido::statespace::SO2StateSpace;
using aikido::statespace::RealVectorStateSpace;

class DifferentiableSubSpaceTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    auto so2 = std::make_shared<SO2StateSpace>();
    constraint =
        std::make_shared<PolynomialConstraint>(Eigen::Vector3d(-1, 0, 1));
    auto rv = constraint->getStateSpace();

    cs = std::make_shared<CompoundStateSpace>(
        std::vector<aikido::statespace::StateSpacePtr>({so2, rv}));
    ds = std::make_shared<DifferentiableSubSpace>(cs, constraint, 1);
  }

  std::shared_ptr<PolynomialConstraint> constraint;
  std::shared_ptr<CompoundStateSpace> cs;
  std::shared_ptr<DifferentiableSubSpace> ds;
};

TEST_F(DifferentiableSubSpaceTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(DifferentiableSubSpace(nullptr, constraint, 1),
               std::invalid_argument);
}

TEST_F(DifferentiableSubSpaceTest, ConstructorThrowsOnNullConstraint)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv = std::make_shared<RealVectorStateSpace>(3);
  auto constraint = std::make_shared<SatisfiedConstraint>(rv);
  auto cs = std::make_shared<CompoundStateSpace>(
      std::vector<aikido::statespace::StateSpacePtr>({so2, rv}));

  EXPECT_THROW(DifferentiableSubSpace(cs, nullptr, 1), std::invalid_argument);
}

TEST_F(DifferentiableSubSpaceTest, ConstructorThrowsOnInvalidIndex)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv = std::make_shared<RealVectorStateSpace>(3);
  auto constraint = std::make_shared<SatisfiedConstraint>(rv);
  auto cs = std::make_shared<CompoundStateSpace>(
      std::vector<aikido::statespace::StateSpacePtr>({so2, rv}));

  EXPECT_THROW(DifferentiableSubSpace(cs, constraint, 2),
               std::invalid_argument);
}

TEST_F(DifferentiableSubSpaceTest, ConstructorThrowsOnMismatchStateSpace)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  auto rv = std::make_shared<RealVectorStateSpace>(3);
  auto constraint = std::make_shared<SatisfiedConstraint>(rv);
  auto cs = std::make_shared<CompoundStateSpace>(
      std::vector<aikido::statespace::StateSpacePtr>({so2, rv}));

  EXPECT_THROW(DifferentiableSubSpace(cs, constraint, 0),
               std::invalid_argument);
}

TEST_F(DifferentiableSubSpaceTest, StateSpace)
{
  EXPECT_EQ(cs, ds->getStateSpace());
}

TEST_F(DifferentiableSubSpaceTest, ConstraintType)
{
  auto ctypes = ds->getConstraintTypes();
  EXPECT_EQ(1, ctypes.size());
  EXPECT_EQ(aikido::constraint::ConstraintType::EQUALITY, ctypes[0]);
}

TEST_F(DifferentiableSubSpaceTest, ConstraintDimension)
{
  EXPECT_EQ(1, ds->getConstraintDimension());
}

TEST_F(DifferentiableSubSpaceTest, ConstraintValue)
{
  auto st = cs->createState();
  auto subSpace = cs->getSubSpace<RealVectorStateSpace>(1);
  auto subState = cs->getSubStateHandle<RealVectorStateSpace>(st, 1);

  subSpace->setValue(subState, aikido::tests::make_vector(2));

  Eigen::VectorXd expected = aikido::tests::make_vector(3);
  EXPECT_TRUE(ds->getValue(st).isApprox(expected));
}

TEST_F(DifferentiableSubSpaceTest, ConstraintJacobian)
{
  auto st = cs->createState();
  auto subSpace = cs->getSubSpace<RealVectorStateSpace>(1);
  auto subState = cs->getSubStateHandle<RealVectorStateSpace>(st, 1);

  subSpace->setValue(subState, aikido::tests::make_vector(2));

  Eigen::VectorXd expected = aikido::tests::make_vector(4);
  EXPECT_TRUE(ds->getJacobian(st).isApprox(expected));
}

TEST_F(DifferentiableSubSpaceTest, ConstraintValueAndJacobian)
{
  auto st = cs->createState();
  auto subSpace = cs->getSubSpace<RealVectorStateSpace>(1);
  auto subState = cs->getSubStateHandle<RealVectorStateSpace>(st, 1);

  subSpace->setValue(subState, aikido::tests::make_vector(2));

  Eigen::VectorXd expectedVal = aikido::tests::make_vector(3);
  Eigen::VectorXd expectedJac = aikido::tests::make_vector(4);
  auto jv = ds->getValueAndJacobian(st);
  EXPECT_TRUE(jv.first.isApprox(expectedVal));
  EXPECT_TRUE(jv.second.isApprox(expectedJac));
}
