#include <gtest/gtest.h>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/constraint/SatisfiedConstraint.hpp>

using aikido::statespace::RealVectorStateSpace;
using aikido::constraint::SatisfiedConstraint;
using Eigen::Vector2d;
using Eigen::Matrix2d;

class SatisfiedConstraintTests : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mStateSpace = std::make_shared<RealVectorStateSpace>(2);
  }

  std::shared_ptr<RealVectorStateSpace> mStateSpace;
};

TEST_F(SatisfiedConstraintTests, constructor_StateSpaceIsNull_Throws)
{
  EXPECT_THROW({
    SatisfiedConstraint(nullptr);
  }, std::invalid_argument);
}

TEST_F(SatisfiedConstraintTests, getStateSpace)
{
  SatisfiedConstraint constraint(mStateSpace);

  EXPECT_EQ(mStateSpace, constraint.getStateSpace());
}

TEST_F(SatisfiedConstraintTests, getConstraintTypes)
{
  SatisfiedConstraint constraint(mStateSpace);

  EXPECT_TRUE(constraint.getConstraintTypes().empty());
}

TEST_F(SatisfiedConstraintTests, isSatisfied_ReturnTrue)
{
  SatisfiedConstraint constraint(mStateSpace);
  auto state = mStateSpace->createState();

  EXPECT_TRUE(constraint.isSatisfied(state));
}

TEST_F(SatisfiedConstraintTests, project_DoesNothing)
{
  SatisfiedConstraint constraint(mStateSpace);
  auto inState = mStateSpace->createState();
  inState.setValue(Vector2d(1., 2.));

  auto outState = mStateSpace->createState();
  EXPECT_TRUE(constraint.project(inState, outState));
  EXPECT_TRUE(inState.getValue().isApprox(outState.getValue()));
}

TEST_F(SatisfiedConstraintTests, getValue_ReturnsZero)
{
  Eigen::Matrix<double, 0, 1> expectedValue;

  SatisfiedConstraint constraint(mStateSpace);
  auto state = mStateSpace->createState();

  auto constraintValue = constraint.getValue(state);
  EXPECT_TRUE(expectedValue.isApprox(constraintValue));
}

TEST_F(SatisfiedConstraintTests, getJacobian_ReturnsZero)
{
  Eigen::Matrix<double, 0, 0> expectedJacobian;

  SatisfiedConstraint constraint(mStateSpace);
  auto state = mStateSpace->createState();

  auto constraintJacobian = constraint.getJacobian(state);
  EXPECT_TRUE(expectedJacobian.isApprox(constraintJacobian));
}

TEST_F(SatisfiedConstraintTests, getValueAndJacobian_ReturnsZero)
{
  Eigen::Matrix<double, 0, 1> expectedValue;
  Eigen::Matrix<double, 0, 0> expectedJacobian;

  SatisfiedConstraint constraint(mStateSpace);
  auto state = mStateSpace->createState();

  auto valueAndJacobian = constraint.getValueAndJacobian(state);
  EXPECT_TRUE(expectedValue.isApprox(valueAndJacobian.first));
  EXPECT_TRUE(expectedJacobian.isApprox(valueAndJacobian.second));
}
