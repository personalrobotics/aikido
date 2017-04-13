#include <gtest/gtest.h>
#include <aikido/constraint/uniform/RnBoxConstraint.hpp>
#include <aikido/distance/RnEuclidean.hpp>
#include <dart/common/StlHelpers.hpp>
#include "SampleGeneratorCoverage.hpp"

using aikido::statespace::R2;
using aikido::statespace::Rn;
using aikido::constraint::R2BoxConstraint;
using aikido::constraint::RnBoxConstraint;
using aikido::constraint::ConstraintType;
using aikido::constraint::SampleGenerator;
using aikido::distance::R2Euclidean;
using aikido::distance::RnEuclidean;
using aikido::util::RNG;
using aikido::util::RNGWrapper;
using dart::common::make_unique;
using Eigen::Vector2d;
using Eigen::Matrix2d;

class RnBoxConstraintTests : public ::testing::Test
{
protected:
  static constexpr size_t NUM_X_TARGETS { 10 };
  static constexpr size_t NUM_Y_TARGETS { 10 };
  static constexpr size_t NUM_SAMPLES { 10000 };
  static constexpr double DISTANCE_THRESHOLD { 0.15 };

  void SetUp() override
  {
    mR2StateSpace = std::make_shared<R2>();
    mRxStateSpace = std::make_shared<Rn>(2);
    mR2Distance = std::make_shared<R2Euclidean>(mR2StateSpace);
    mRxDistance = std::make_shared<RnEuclidean>(mRxStateSpace);
    mRng = make_unique<RNGWrapper<std::default_random_engine>>(0);

    mLowerLimits = Vector2d(-1., 1.);
    mUpperLimits = Vector2d( 1., 2.);

    mGoodValues.resize(3);
    mGoodValues[0] = Vector2d(-0.9, 1.1);
    mGoodValues[1] = Vector2d( 0.0, 1.5);
    mGoodValues[2] = Vector2d( 0.9, 1.9);

    mBadValues.resize(8);
    mBadValues[0] = Vector2d(-1.1, 1.5);
    mBadValues[1] = Vector2d( 1.1, 1.5);
    mBadValues[2] = Vector2d( 0.0, 0.9);
    mBadValues[3] = Vector2d( 0.0, 2.1);
    mBadValues[4] = Vector2d(-1.1, 0.9);
    mBadValues[5] = Vector2d(-1.1, 2.1);
    mBadValues[6] = Vector2d( 1.1, 0.9);
    mBadValues[7] = Vector2d( 1.1, 2.1);

    mTargets.clear();
    mTargets.reserve(NUM_X_TARGETS * NUM_Y_TARGETS);

    for (size_t ix = 0; ix < NUM_X_TARGETS; ++ix)
    {
      auto xRatio = static_cast<double>(ix) / (NUM_X_TARGETS - 1);
      auto x = (1 - xRatio) * mLowerLimits[0] + xRatio * mUpperLimits[0];

      for (size_t iy = 0; iy < NUM_Y_TARGETS; ++iy)
      {
        auto yRatio = static_cast<double>(iy) / (NUM_Y_TARGETS - 1);
        auto y = (1 - yRatio) * mLowerLimits[1] + yRatio * mUpperLimits[1];

        auto state = mR2StateSpace->createState();
        state.setValue(Vector2d(x, y));

        mTargets.emplace_back(std::move(state));
      }
    }
  }

  std::unique_ptr<RNG> mRng;
  std::shared_ptr<R2> mR2StateSpace;
  std::shared_ptr<Rn> mRxStateSpace;
  std::shared_ptr<R2Euclidean> mR2Distance;
  std::shared_ptr<RnEuclidean> mRxDistance;

  Eigen::Vector2d mLowerLimits;
  Eigen::Vector2d mUpperLimits;

  std::vector<Eigen::Vector2d,
    Eigen::aligned_allocator<Eigen::Vector2d>> mGoodValues;
  std::vector<Eigen::Vector2d,
    Eigen::aligned_allocator<Eigen::Vector2d>> mBadValues;

  std::vector<R2::ScopedState> mTargets;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_constructor_StateSpaceIsNull_Throws)
{
  EXPECT_THROW({
    R2BoxConstraint(nullptr, mRng->clone(), mLowerLimits, mUpperLimits);
  }, std::invalid_argument);
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_constructor_StateSpaceIsNull_Throws)
{
  EXPECT_THROW({
    RnBoxConstraint(nullptr, mRng->clone(), mLowerLimits, mUpperLimits);
  }, std::invalid_argument);
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_constructor_RNGIsNull_DoesNotThrow)
{
  EXPECT_NO_THROW({
    R2BoxConstraint(mR2StateSpace, nullptr, mLowerLimits, mUpperLimits);
  });
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_constructor_RNGIsNull_DoesNotThrow)
{
  EXPECT_NO_THROW({
    RnBoxConstraint(mRxStateSpace, nullptr, mLowerLimits, mUpperLimits);
  });
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_constructor_LowersLimitExceedsUpperLimits_Throws)
{
  Eigen::Vector2d badLowerLimits(1., 0.);
  Eigen::Vector2d badUpperLimits(0., 1.);

  EXPECT_THROW({
    R2BoxConstraint(
      mR2StateSpace, mRng->clone(), badLowerLimits, badUpperLimits);
  }, std::invalid_argument);
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_constructor_LowersLimitExceedsUpperLimits_Throws)
{
  Eigen::Vector2d badLowerLimits(1., 0.);
  Eigen::Vector2d badUpperLimits(0., 1.);

  EXPECT_THROW({
    RnBoxConstraint(
      mRxStateSpace, mRng->clone(), badLowerLimits, badUpperLimits);
  }, std::invalid_argument);
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_getStateSpace)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  EXPECT_EQ(mR2StateSpace, constraint.getStateSpace());
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_getStateSpace)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  EXPECT_EQ(mRxStateSpace, constraint.getStateSpace());
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_getConstraintDimension)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  EXPECT_EQ(2, constraint.getConstraintDimension());
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_getConstraintDimension)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  EXPECT_EQ(2, constraint.getConstraintDimension());
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_getConstraintTypes)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);
  auto constraintTypes = constraint.getConstraintTypes();

  ASSERT_EQ(2, constraintTypes.size());
  EXPECT_EQ(ConstraintType::INEQUALITY, constraintTypes[0]);
  EXPECT_EQ(ConstraintType::INEQUALITY, constraintTypes[1]);
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_getConstraintTypes)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);
  auto constraintTypes = constraint.getConstraintTypes();

  ASSERT_EQ(2, constraintTypes.size());
  EXPECT_EQ(ConstraintType::INEQUALITY, constraintTypes[0]);
  EXPECT_EQ(ConstraintType::INEQUALITY, constraintTypes[1]);
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_isSatisfied_SatisfiesConstraint_ReturnsTrue)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mR2StateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    state.setValue(value);
    EXPECT_TRUE(constraint.isSatisfied(state));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_isSatisfied_SatisfiesConstraint_ReturnsTrue)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mRxStateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    state.setValue(value);
    EXPECT_TRUE(constraint.isSatisfied(state));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_isSatisfied_DoesNotSatisfyConstraint_ReturnsFalse)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mR2StateSpace->createState();

  for (const auto& value : mBadValues)
  {
    state.setValue(value);
    EXPECT_FALSE(constraint.isSatisfied(state));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_isSatisfied_DoesNotSatisfyConstraint_ReturnsFalse)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mRxStateSpace->createState();

  for (const auto& value : mBadValues)
  {
    state.setValue(value);
    EXPECT_FALSE(constraint.isSatisfied(state));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_project_SatisfiesConstraint_DoesNothing)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto inState = mR2StateSpace->createState();
  auto outState = mR2StateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    inState.setValue(value);
    EXPECT_TRUE(constraint.project(inState, outState));
    EXPECT_TRUE(value.isApprox(outState.getValue()));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_project_SatisfiesConstraint_DoesNothing)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto inState = mRxStateSpace->createState();
  auto outState = mRxStateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    inState.setValue(value);
    EXPECT_TRUE(constraint.project(inState, outState));
    EXPECT_TRUE(value.isApprox(outState.getValue()));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_project_DoesNotSatisfyConstraint_Projects)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto inState = mR2StateSpace->createState();
  auto outState = mR2StateSpace->createState();

  for (const auto& value : mBadValues)
  {
    inState.setValue(value);
    EXPECT_TRUE(constraint.project(inState, outState));
    EXPECT_TRUE(constraint.isSatisfied(outState));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_project_DoesNotSatisfyConstraint_Projects)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto inState = mRxStateSpace->createState();
  auto outState = mRxStateSpace->createState();

  for (const auto& value : mBadValues)
  {
    inState.setValue(value);
    EXPECT_TRUE(constraint.project(inState, outState));
    EXPECT_TRUE(constraint.isSatisfied(outState));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_getValue_SatisfiesConstraint_ReturnsZero)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mR2StateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    state.setValue(value);
    Eigen::VectorXd constraintValue;
    constraint.getValue(state, constraintValue);
    EXPECT_TRUE(Vector2d::Zero().isApprox(constraintValue));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_getValue_SatisfiesConstraint_ReturnsZero)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mRxStateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    state.setValue(value);
    Eigen::VectorXd constraintValue;
    constraint.getValue(state, constraintValue);
    EXPECT_TRUE(Vector2d::Zero().isApprox(constraintValue));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_getValue_DoesNotSatisfyConstraint_ReturnsNonZero)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mR2StateSpace->createState();

  // TODO: Check the sign.
  // TODO: Check which elements are non-zero.

  for (const auto& value : mBadValues)
  {
    state.setValue(value);
    Eigen::VectorXd constraintValue;
    constraint.getValue(state, constraintValue);
    EXPECT_FALSE(Vector2d::Zero().isApprox(constraintValue));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_getValue_DoesNotSatisfyConstraint_ReturnsNonZero)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mRxStateSpace->createState();

  // TODO: Check the sign.
  // TODO: Check which elements are non-zero.

  for (const auto& value : mBadValues)
  {
    state.setValue(value);
    Eigen::VectorXd constraintValue;
    constraint.getValue(state, constraintValue);
    EXPECT_FALSE(Vector2d::Zero().isApprox(constraintValue));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_getJacobian_SatisfiesConstraint_ReturnsZero)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mR2StateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    state.setValue(value);
    Eigen::MatrixXd jacobian;
    constraint.getJacobian(state, jacobian);
    EXPECT_TRUE(Matrix2d::Zero().isApprox(jacobian));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_getJacobian_SatisfiesConstraint_ReturnsZero)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mRxStateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    state.setValue(value);
    Eigen::MatrixXd jacobian;
    constraint.getJacobian(state, jacobian);
    EXPECT_TRUE(Matrix2d::Zero().isApprox(jacobian));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_getJacobian_DoesNotSatisfyConstraint_ReturnsNonZero)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  // TODO: Check the sign.
  // TODO: Check which elements are non-zero.

  auto state = mR2StateSpace->createState();

  for (const auto& value : mBadValues)
  {
    state.setValue(value);
    Eigen::MatrixXd jacobian;
    constraint.getJacobian(state, jacobian);
    EXPECT_FALSE(Matrix2d::Zero().isApprox(jacobian));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_getJacobian_DoesNotSatisfyConstraint_ReturnsNonZero)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  // TODO: Check the sign.
  // TODO: Check which elements are non-zero.

  auto state = mRxStateSpace->createState();

  for (const auto& value : mBadValues)
  {
    state.setValue(value);
    Eigen::MatrixXd jacobian;
    constraint.getJacobian(state, jacobian);
    EXPECT_FALSE(Matrix2d::Zero().isApprox(jacobian));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_getValueAndJacobian_SatisfiesConstraint_ReturnsZero)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mR2StateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    state.setValue(value);
    Eigen::VectorXd constraintValue;
    Eigen::MatrixXd constraintJac;

    constraint.getValueAndJacobian(state, constraintValue, constraintJac);

    EXPECT_TRUE(Vector2d::Zero().isApprox(constraintValue));
    EXPECT_TRUE(Matrix2d::Zero().isApprox(constraintJac));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_getValueAndJacobian_SatisfiesConstraint_ReturnsZero)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mRxStateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    state.setValue(value);
    Eigen::VectorXd constraintValue;
    Eigen::MatrixXd constraintJac;

    constraint.getValueAndJacobian(state, constraintValue, constraintJac);

    EXPECT_TRUE(Vector2d::Zero().isApprox(constraintValue));
    EXPECT_TRUE(Matrix2d::Zero().isApprox(constraintJac));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_getValueAndJacobian_DoesNotSatisfyConstraint_ReturnsNonZero)
{
  R2BoxConstraint constraint(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mR2StateSpace->createState();

  for (const auto& value : mBadValues)
  {
    state.setValue(value);
    Eigen::VectorXd constraintValue;
    Eigen::MatrixXd constraintJac;

    constraint.getValueAndJacobian(state, constraintValue, constraintJac);

    EXPECT_FALSE(Vector2d::Zero().isApprox(constraintValue));
    EXPECT_FALSE(Matrix2d::Zero().isApprox(constraintJac));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_getValueAndJacobian_DoesNotSatisfyConstraint_ReturnsNonZero)
{
  RnBoxConstraint constraint(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mRxStateSpace->createState();

  for (const auto& value : mBadValues)
  {
    state.setValue(value);
    Eigen::VectorXd constraintValue;
    Eigen::MatrixXd constraintJac;

    constraint.getValueAndJacobian(state, constraintValue, constraintJac);

    EXPECT_FALSE(Vector2d::Zero().isApprox(constraintValue));
    EXPECT_FALSE(Matrix2d::Zero().isApprox(constraintJac));
  }
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_createSampleGenerator)
{
  auto constraint = std::make_shared<R2BoxConstraint>(
    mR2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto generator = constraint->createSampleGenerator();
  EXPECT_EQ(mR2StateSpace, generator->getStateSpace());

  auto result = SampleGeneratorCoverage(*generator, *mR2Distance,
    std::begin(mTargets), std::end(mTargets), DISTANCE_THRESHOLD, NUM_SAMPLES);
  ASSERT_TRUE(result);
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_createSampleGenerator)
{
  auto constraint = std::make_shared<RnBoxConstraint>(
    mRxStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto generator = constraint->createSampleGenerator();
  EXPECT_EQ(mRxStateSpace, generator->getStateSpace());

  auto result = SampleGeneratorCoverage(*generator, *mRxDistance,
    std::begin(mTargets), std::end(mTargets), DISTANCE_THRESHOLD, NUM_SAMPLES);
  ASSERT_TRUE(result);
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_createSampleGenerator_RNGIsNull_Throws)
{
  // We need to use make_shared here because createSampleGenerator calls
  // shared_from_this, provided by enable_shared_from_this.
  auto constraint = std::make_shared<R2BoxConstraint>(
    mR2StateSpace, nullptr, mLowerLimits, mUpperLimits);

  EXPECT_THROW({
    constraint->createSampleGenerator();
      }, std::invalid_argument);
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_createSampleGenerator_RNGIsNull_Throws)
{
  // We need to use make_shared here because createSampleGenerator calls
  // shared_from_this, provided by enable_shared_from_this.
  auto constraint = std::make_shared<RnBoxConstraint>(
    mRxStateSpace, nullptr, mLowerLimits, mUpperLimits);

  EXPECT_THROW({
    constraint->createSampleGenerator();
      }, std::invalid_argument);
}

//==============================================================================
TEST_F(RnBoxConstraintTests, R2_createSampleGenerator_Unbounded_Throws)
{
  Vector2d noLowerBound = mLowerLimits;
  noLowerBound[0] = -std::numeric_limits<double>::infinity();

  Vector2d noUpperBound = mUpperLimits;
  noUpperBound[1] = std::numeric_limits<double>::infinity();

  // We need to use make_shared here because createSampleGenerator calls
  // shared_from_this, provided by enable_shared_from_this.
  auto unbounded1 = std::make_shared<R2BoxConstraint>(
    mR2StateSpace, mRng->clone(), noLowerBound, mUpperLimits);
  EXPECT_THROW({
    unbounded1->createSampleGenerator();
  }, std::runtime_error);

  auto unbounded2 = std::make_shared<R2BoxConstraint>(
    mR2StateSpace, mRng->clone(), mLowerLimits, noUpperBound);
  EXPECT_THROW({
    unbounded2->createSampleGenerator();
  }, std::runtime_error);
}

//==============================================================================
TEST_F(RnBoxConstraintTests, Rx_createSampleGenerator_Unbounded_Throws)
{
  Vector2d noLowerBound = mLowerLimits;
  noLowerBound[0] = -std::numeric_limits<double>::infinity();

  Vector2d noUpperBound = mUpperLimits;
  noUpperBound[1] = std::numeric_limits<double>::infinity();

  // We need to use make_shared here because createSampleGenerator calls
  // shared_from_this, provided by enable_shared_from_this.
  auto unbounded1 = std::make_shared<RnBoxConstraint>(
    mRxStateSpace, mRng->clone(), noLowerBound, mUpperLimits);
  EXPECT_THROW({
    unbounded1->createSampleGenerator();
  }, std::runtime_error);

  auto unbounded2 = std::make_shared<RnBoxConstraint>(
    mRxStateSpace, mRng->clone(), mLowerLimits, noUpperBound);
  EXPECT_THROW({
    unbounded2->createSampleGenerator();
  }, std::runtime_error);
}
