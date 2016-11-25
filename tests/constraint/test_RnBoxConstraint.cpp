#include <gtest/gtest.h>
#include <aikido/constraint/uniform/RnBoxConstraint.hpp>
#include <aikido/distance/RnEuclidean.hpp>
#include <dart/common/StlHelpers.hpp>
#include "SampleGeneratorCoverage.hpp"

using aikido::statespace::Rn;
using aikido::constraint::RnBoxConstraint;
using aikido::constraint::ConstraintType;
using aikido::constraint::SampleGenerator;
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
    mStateSpace = std::make_shared<Rn>(2);
    mDistance = std::make_shared<RnEuclidean>(mStateSpace);
    mRng = make_unique<RNGWrapper<std::default_random_engine>>(0);

    mLowerLimits = Vector2d(-1., 1.);
    mUpperLimits = Vector2d( 1., 2.);

    mGoodValues = {
      Vector2d(-0.9, 1.1),
      Vector2d( 0.0, 1.5),
      Vector2d( 0.9, 1.9)
    };
    mBadValues = {
      Vector2d(-1.1, 1.5),
      Vector2d( 1.1, 1.5),
      Vector2d( 0.0, 0.9),
      Vector2d( 0.0, 2.1),
      Vector2d(-1.1, 0.9),
      Vector2d(-1.1, 2.1),
      Vector2d( 1.1, 0.9),
      Vector2d( 1.1, 2.1)
    };

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

        auto state = mStateSpace->createState();
        state.setValue(Vector2d(x, y));

        mTargets.emplace_back(std::move(state));
      }
    }
  }

  std::unique_ptr<RNG> mRng;
  std::shared_ptr<Rn> mStateSpace;
  std::shared_ptr<RnEuclidean> mDistance;

  Eigen::Vector2d mLowerLimits;
  Eigen::Vector2d mUpperLimits;

  std::vector<Eigen::Vector2d,
    Eigen::aligned_allocator<Eigen::Vector2d>> mGoodValues;
  std::vector<Eigen::Vector2d,
    Eigen::aligned_allocator<Eigen::Vector2d>> mBadValues;

  std::vector<Rn::ScopedState> mTargets;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(RnBoxConstraintTests, constructor_StateSpaceIsNull_Throws)
{
  EXPECT_THROW({
    RnBoxConstraint(nullptr, mRng->clone(), mLowerLimits, mUpperLimits);
  }, std::invalid_argument);
}

TEST_F(RnBoxConstraintTests, constructor_RNGIsNull_DoesNotThrow)
{
  EXPECT_NO_THROW({
    RnBoxConstraint(mStateSpace, nullptr, mLowerLimits, mUpperLimits);
  });
}

TEST_F(RnBoxConstraintTests, constructor_LowerLimitsWrongSize_Throws)
{
  Eigen::Vector3d badLowerLimits(0., 0., 0.);
  Eigen::Vector2d goodUpperLimits(1., 1.);

  EXPECT_THROW({
    RnBoxConstraint(
      mStateSpace, mRng->clone(), badLowerLimits, goodUpperLimits);
  }, std::invalid_argument);
}

TEST_F(RnBoxConstraintTests, constructor_UpperLimitsWrongSize_Throws)
{
  Eigen::Vector2d goodLowerLimits(0., 0.);
  Eigen::Vector3d badUpperLimits(1., 1., 1.);

  EXPECT_THROW({
    RnBoxConstraint(
      mStateSpace, mRng->clone(), goodLowerLimits, badUpperLimits);
  }, std::invalid_argument);
}

TEST_F(RnBoxConstraintTests, constructor_LowersLimitExceedsUpperLimits_Throws)
{
  Eigen::Vector2d badLowerLimits(1., 0.);
  Eigen::Vector2d badUpperLimits(0., 1.);

  EXPECT_THROW({
    RnBoxConstraint(
      mStateSpace, mRng->clone(), badLowerLimits, badUpperLimits);
  }, std::invalid_argument);
}

TEST_F(RnBoxConstraintTests, getStateSpace)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  EXPECT_EQ(mStateSpace, constraint.getStateSpace());
}

TEST_F(RnBoxConstraintTests, getConstraintDimension)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  EXPECT_EQ(2, constraint.getConstraintDimension());
}

TEST_F(RnBoxConstraintTests, getConstraintTypes)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);
  auto constraintTypes = constraint.getConstraintTypes();

  ASSERT_EQ(2, constraintTypes.size());
  EXPECT_EQ(ConstraintType::INEQUALITY, constraintTypes[0]);
  EXPECT_EQ(ConstraintType::INEQUALITY, constraintTypes[1]);
}

TEST_F(RnBoxConstraintTests, isSatisfied_SatisfiesConstraint_ReturnsTrue)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mStateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    state.setValue(value);
    EXPECT_TRUE(constraint.isSatisfied(state));
  }
}


TEST_F(RnBoxConstraintTests, isSatisfied_DoesNotSatisfyConstraint_ReturnsFalse)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mStateSpace->createState();

  for (const auto& value : mBadValues)
  {
    state.setValue(value);
    EXPECT_FALSE(constraint.isSatisfied(state));
  }
}

TEST_F(RnBoxConstraintTests, project_SatisfiesConstraint_DoesNothing)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto inState = mStateSpace->createState();
  auto outState = mStateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    inState.setValue(value);
    EXPECT_TRUE(constraint.project(inState, outState));
    EXPECT_TRUE(value.isApprox(outState.getValue()));
  }
}

TEST_F(RnBoxConstraintTests, project_DoesNotSatisfyConstraint_Projects)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto inState = mStateSpace->createState();
  auto outState = mStateSpace->createState();

  for (const auto& value : mBadValues)
  {
    inState.setValue(value);
    EXPECT_TRUE(constraint.project(inState, outState));
    EXPECT_TRUE(constraint.isSatisfied(outState));
  }
}

TEST_F(RnBoxConstraintTests, getValue_SatisfiesConstraint_ReturnsZero)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mStateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    state.setValue(value);
    Eigen::VectorXd constraintValue;
    constraint.getValue(state, constraintValue);
    EXPECT_TRUE(Vector2d::Zero().isApprox(constraintValue));
  }
}

TEST_F(RnBoxConstraintTests, getValue_DoesNotSatisfyConstraint_ReturnsNonZero)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mStateSpace->createState();

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

TEST_F(RnBoxConstraintTests, getJacobian_SatisfiesConstraint_ReturnsZero)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mStateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    state.setValue(value);
    Eigen::MatrixXd jacobian;
    constraint.getJacobian(state, jacobian);
    EXPECT_TRUE(Matrix2d::Zero().isApprox(jacobian));
  }
}

TEST_F(RnBoxConstraintTests, getJacobian_DoesNotSatisfyConstraint_ReturnsNonZero)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  // TODO: Check the sign.
  // TODO: Check which elements are non-zero.

  auto state = mStateSpace->createState();

  for (const auto& value : mBadValues)
  {
    state.setValue(value);
    Eigen::MatrixXd jacobian;
    constraint.getJacobian(state, jacobian);
    EXPECT_FALSE(Matrix2d::Zero().isApprox(jacobian));
  }
}

TEST_F(RnBoxConstraintTests, getValueAndJacobian_SatisfiesConstraint_ReturnsZero)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mStateSpace->createState();

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

TEST_F(RnBoxConstraintTests, getValueAndJacobian_DoesNotSatisfyConstraint_ReturnsNonZero)
{
  RnBoxConstraint constraint(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mStateSpace->createState();

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

TEST_F(RnBoxConstraintTests, createSampleGenerator)
{
  auto constraint = std::make_shared<RnBoxConstraint>(
    mStateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto generator = constraint->createSampleGenerator();
  EXPECT_EQ(mStateSpace, generator->getStateSpace());

  auto result = SampleGeneratorCoverage(*generator, *mDistance,
    std::begin(mTargets), std::end(mTargets), DISTANCE_THRESHOLD, NUM_SAMPLES);
  ASSERT_TRUE(result);
}

TEST_F(RnBoxConstraintTests, createSampleGenerator_RNGIsNull_Throws)
{
  // We need to use make_shared here because createSampleGenerator calls
  // shared_from_this, provided by enable_shared_from_this.
  auto constraint = std::make_shared<RnBoxConstraint>(
    mStateSpace, nullptr, mLowerLimits, mUpperLimits);

  EXPECT_THROW({
    constraint->createSampleGenerator();
      }, std::invalid_argument);
}

TEST_F(RnBoxConstraintTests, createSampleGenerator_Unbounded_Throws)
{
  Vector2d noLowerBound = mLowerLimits;
  noLowerBound[0] = -std::numeric_limits<double>::infinity();

  Vector2d noUpperBound = mUpperLimits;
  noUpperBound[1] = std::numeric_limits<double>::infinity();

  // We need to use make_shared here because createSampleGenerator calls
  // shared_from_this, provided by enable_shared_from_this.
  auto unbounded1 = std::make_shared<RnBoxConstraint>(
    mStateSpace, mRng->clone(), noLowerBound, mUpperLimits);
  EXPECT_THROW({
    unbounded1->createSampleGenerator();
  }, std::runtime_error);

  auto unbounded2 = std::make_shared<RnBoxConstraint>(
    mStateSpace, mRng->clone(), mLowerLimits, noUpperBound);
  EXPECT_THROW({
    unbounded2->createSampleGenerator();
  }, std::runtime_error);
}
