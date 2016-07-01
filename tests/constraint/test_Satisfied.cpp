#include <gtest/gtest.h>
#include <aikido/statespace/Rn.hpp>
#include <aikido/constraint/Satisfied.hpp>

using aikido::statespace::Rn;
using aikido::constraint::Satisfied;
using Eigen::Vector2d;
using Eigen::Matrix2d;

class SatisfiedTests : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mStateSpace = std::make_shared<Rn>(2);
  }

  std::shared_ptr<Rn> mStateSpace;
};

TEST_F(SatisfiedTests, constructor_StateSpaceIsNull_Throws)
{
  EXPECT_THROW({
    Satisfied(nullptr);
  }, std::invalid_argument);
}

TEST_F(SatisfiedTests, getStateSpace)
{
  Satisfied constraint(mStateSpace);

  EXPECT_EQ(mStateSpace, constraint.getStateSpace());
}

TEST_F(SatisfiedTests, getConstraintTypes)
{
  Satisfied constraint(mStateSpace);

  EXPECT_TRUE(constraint.getConstraintTypes().empty());
}

TEST_F(SatisfiedTests, isSatisfied_ReturnTrue)
{
  Satisfied constraint(mStateSpace);
  auto state = mStateSpace->createState();

  EXPECT_TRUE(constraint.isSatisfied(state));
}

TEST_F(SatisfiedTests, project_DoesNothing)
{
  Satisfied constraint(mStateSpace);
  auto inState = mStateSpace->createState();
  inState.setValue(Vector2d(1., 2.));

  auto outState = mStateSpace->createState();
  EXPECT_TRUE(constraint.project(inState, outState));
  EXPECT_TRUE(inState.getValue().isApprox(outState.getValue()));
}

TEST_F(SatisfiedTests, getValue_ReturnsZero)
{
  Eigen::Matrix<double, 0, 1> expectedValue;

  Satisfied constraint(mStateSpace);
  auto state = mStateSpace->createState();

  Eigen::VectorXd constraintValue;
  constraint.getValue(state, constraintValue);
  EXPECT_TRUE(expectedValue.isApprox(constraintValue));
}

TEST_F(SatisfiedTests, getJacobian_ReturnsZero)
{
  Eigen::Matrix<double, 0, 0> expectedJacobian;

  Satisfied constraint(mStateSpace);
  auto state = mStateSpace->createState();

  Eigen::MatrixXd constraintJacobian;
  constraint.getJacobian(state, constraintJacobian);
  EXPECT_TRUE(expectedJacobian.isApprox(constraintJacobian));
}

TEST_F(SatisfiedTests, getValueAndJacobian_ReturnsZero)
{
  Eigen::Matrix<double, 0, 1> expectedValue;
  Eigen::Matrix<double, 0, 0> expectedJacobian;

  Satisfied constraint(mStateSpace);
  auto state = mStateSpace->createState();

  Eigen::VectorXd value;
  Eigen::MatrixXd jacobian;

  constraint.getValueAndJacobian(state, value, jacobian);
  EXPECT_TRUE(expectedValue.isApprox(value));
  EXPECT_TRUE(expectedJacobian.isApprox(jacobian));
}
