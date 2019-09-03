#include <dart/common/Memory.hpp>
#include <gtest/gtest.h>
#include <aikido/common/memory.hpp>
#include <aikido/constraint/uniform/SE2BoxConstraint.hpp>
#include <aikido/distance/SE2Weighted.hpp>
#include "SampleGeneratorCoverage.hpp"

using aikido::common::RNG;
using aikido::common::RNGWrapper;
using aikido::constraint::ConstraintType;
using aikido::constraint::SampleGenerator;
using aikido::constraint::uniform::SE2BoxConstraint;
using aikido::distance::SE2Weighted;
using aikido::statespace::SE2;
using Eigen::Isometry2d;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;

class SE2BoxConstraintTests : public ::testing::Test
{
protected:
#ifndef NDEBUG
  static constexpr std::size_t NUM_X_TARGETS{5};
  static constexpr std::size_t NUM_Y_TARGETS{5};
  static constexpr std::size_t NUM_A_TARGETS{5};
  static constexpr std::size_t NUM_SAMPLES{125};
  static constexpr double DISTANCE_THRESHOLD{1.5};
#else
  static constexpr std::size_t NUM_X_TARGETS{10};
  static constexpr std::size_t NUM_Y_TARGETS{10};
  static constexpr std::size_t NUM_A_TARGETS{10};
  static constexpr std::size_t NUM_SAMPLES{1000};
  static constexpr double DISTANCE_THRESHOLD{0.8};
#endif

  void SetUp() override
  {
    mSE2StateSpace = dart::common::make_aligned_shared<SE2>();
    mSE2Distance
        = dart::common::make_aligned_shared<SE2Weighted>(mSE2StateSpace);
    mRng
        = ::aikido::common::make_unique<RNGWrapper<std::default_random_engine>>(
            0);

    mLowerLimits = Vector2d(-1., 1.);
    mUpperLimits = Vector2d(1., 2.);

    mGoodValues.resize(3);
    mGoodValues[0] = Vector3d(-0.9, 1.1, 0.0);
    mGoodValues[1] = Vector3d(0.0, 1.5, M_PI_2);
    mGoodValues[2] = Vector3d(0.9, 1.9, -M_PI_4);

    mBadValues.resize(8);
    mBadValues[0] = Vector3d(-1.1, 1.5, 0.0);
    mBadValues[1] = Vector3d(1.1, 1.5, M_PI_2);
    mBadValues[2] = Vector3d(0.0, 0.9, M_PI);
    mBadValues[3] = Vector3d(0.0, 2.1, 3.0 * M_PI_2);
    mBadValues[4] = Vector3d(-1.1, 0.9, 2.0 * M_PI);
    mBadValues[5] = Vector3d(-1.1, 2.1, -M_PI_2);
    mBadValues[6] = Vector3d(1.1, 0.9, -M_PI);
    mBadValues[7] = Vector3d(1.1, 2.1, -3.0 * M_PI_2);

    mTargets.clear();
    mTargets.reserve(NUM_A_TARGETS * NUM_X_TARGETS * NUM_Y_TARGETS);

    for (std::size_t ia = 0; ia < NUM_A_TARGETS; ++ia)
    {
      auto aRatio = static_cast<double>(ia) / (NUM_A_TARGETS - 1);
      auto a = (1 - aRatio) * (-M_PI) + aRatio * M_PI;

      for (std::size_t ix = 0; ix < NUM_X_TARGETS; ++ix)
      {
        auto xRatio = static_cast<double>(ix) / (NUM_X_TARGETS - 1);
        auto x = (1 - xRatio) * mLowerLimits[0] + xRatio * mUpperLimits[0];

        for (std::size_t iy = 0; iy < NUM_Y_TARGETS; ++iy)
        {
          auto yRatio = static_cast<double>(iy) / (NUM_Y_TARGETS - 1);
          auto y = (1 - yRatio) * mLowerLimits[1] + yRatio * mUpperLimits[1];

          auto state = mSE2StateSpace->createState();
          Isometry2d pose = Eigen::Isometry2d::Identity();
          pose = pose.translate(Vector2d(x, y)).rotate(a);
          state.setIsometry(pose);

          mTargets.emplace_back(std::move(state));
        }
      }
    }
  }

  std::unique_ptr<RNG> mRng;
  std::shared_ptr<SE2> mSE2StateSpace;
  std::shared_ptr<SE2Weighted> mSE2Distance;

  Eigen::Vector2d mLowerLimits;
  Eigen::Vector2d mUpperLimits;

  std::vector<Vector3d, Eigen::aligned_allocator<Vector3d>> mGoodValues;
  std::vector<Vector3d, Eigen::aligned_allocator<Vector3d>> mBadValues;

  std::vector<SE2::ScopedState> mTargets;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//==============================================================================
TEST_F(SE2BoxConstraintTests, ThrowsOnNullStateSpace)
{
  EXPECT_THROW(
      { SE2BoxConstraint(nullptr, mRng->clone(), mLowerLimits, mUpperLimits); },
      std::invalid_argument);
}

//==============================================================================
TEST_F(SE2BoxConstraintTests, DoesNotThrowOnNullRNG)
{
  EXPECT_NO_THROW({
    SE2BoxConstraint(mSE2StateSpace, nullptr, mLowerLimits, mUpperLimits);
  });
}

//==============================================================================
TEST_F(SE2BoxConstraintTests, ThrowsOnLowersLimitExceedsUpperLimits)
{
  Eigen::Vector2d badLowerLimits(1., 0.);
  Eigen::Vector2d badUpperLimits(0., 1.);

  EXPECT_THROW(
      {
        SE2BoxConstraint(
            mSE2StateSpace, mRng->clone(), badLowerLimits, badUpperLimits);
      },
      std::invalid_argument);
}

//==============================================================================
TEST_F(SE2BoxConstraintTests, getStateSpace)
{
  SE2BoxConstraint constraint(
      mSE2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  EXPECT_EQ(mSE2StateSpace, constraint.getStateSpace());
}

//==============================================================================
TEST_F(SE2BoxConstraintTests, isSatisfiedTrue)
{
  SE2BoxConstraint constraint(
      mSE2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mSE2StateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    Isometry2d pose = Eigen::Isometry2d::Identity();
    pose = pose.translate(Vector2d(value[0], value[1])).rotate(value[2]);
    state.setIsometry(pose);
    EXPECT_TRUE(constraint.isSatisfied(state));
  }
}

//==============================================================================
TEST_F(SE2BoxConstraintTests, isSatisfiedFalse)
{
  SE2BoxConstraint constraint(
      mSE2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto state = mSE2StateSpace->createState();

  for (const auto& value : mBadValues)
  {
    Isometry2d pose = Eigen::Isometry2d::Identity();
    pose = pose.translate(Vector2d(value[0], value[1])).rotate(value[2]);
    state.setIsometry(pose);
    EXPECT_FALSE(constraint.isSatisfied(state));
  }
}

//==============================================================================
TEST_F(SE2BoxConstraintTests, projectSatisfiedConstraintDoesNothing)
{
  SE2BoxConstraint constraint(
      mSE2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto inState = mSE2StateSpace->createState();
  auto outState = mSE2StateSpace->createState();

  for (const auto& value : mGoodValues)
  {
    Isometry2d pose = Eigen::Isometry2d::Identity();
    pose = pose.translate(Vector2d(value[0], value[1])).rotate(value[2]);

    inState.setIsometry(pose);
    EXPECT_TRUE(constraint.project(inState, outState));
    EXPECT_TRUE(pose.isApprox(outState.getIsometry()));
  }
}

//==============================================================================
TEST_F(SE2BoxConstraintTests, projectUnSatisfiedConstraintProjects)
{
  SE2BoxConstraint constraint(
      mSE2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto inState = mSE2StateSpace->createState();
  auto outState = mSE2StateSpace->createState();

  for (const auto& value : mBadValues)
  {
    Isometry2d pose = Eigen::Isometry2d::Identity();
    pose = pose.translate(Vector2d(value[0], value[1])).rotate(value[2]);

    inState.setIsometry(pose);
    EXPECT_TRUE(constraint.project(inState, outState));
    EXPECT_TRUE(constraint.isSatisfied(outState));
  }
}

//==============================================================================
TEST_F(SE2BoxConstraintTests, createSampleGenerator)
{
  auto constraint = dart::common::make_aligned_shared<SE2BoxConstraint>(
      mSE2StateSpace, mRng->clone(), mLowerLimits, mUpperLimits);

  auto generator = constraint->createSampleGenerator();
  EXPECT_EQ(mSE2StateSpace, generator->getStateSpace());

  auto result = SampleGeneratorCoverage(
      *generator,
      *mSE2Distance,
      std::begin(mTargets),
      std::end(mTargets),
      DISTANCE_THRESHOLD,
      NUM_SAMPLES);
  ASSERT_TRUE(result);
}

//==============================================================================
TEST_F(SE2BoxConstraintTests, createSampleGeneratorThrowOnNullRNG)
{
  // We need to use make_shared here because createSampleGenerator calls
  // shared_from_this, provided by enable_shared_from_this.
  auto constraint = dart::common::make_aligned_shared<SE2BoxConstraint>(
      mSE2StateSpace, nullptr, mLowerLimits, mUpperLimits);

  EXPECT_THROW({ constraint->createSampleGenerator(); }, std::invalid_argument);
}

//==============================================================================
TEST_F(SE2BoxConstraintTests, createSampleGeneratorThrowsIfUnbounded)
{
  Vector2d noLowerBound = mLowerLimits;
  noLowerBound[0] = -std::numeric_limits<double>::infinity();

  Vector2d noUpperBound = mUpperLimits;
  noUpperBound[1] = std::numeric_limits<double>::infinity();

  // We need to use make_shared here because createSampleGenerator calls
  // shared_from_this, provided by enable_shared_from_this.
  auto unbounded1 = dart::common::make_aligned_shared<SE2BoxConstraint>(
      mSE2StateSpace, mRng->clone(), noLowerBound, mUpperLimits);
  EXPECT_THROW({ unbounded1->createSampleGenerator(); }, std::runtime_error);

  auto unbounded2 = dart::common::make_aligned_shared<SE2BoxConstraint>(
      mSE2StateSpace, mRng->clone(), mLowerLimits, noUpperBound);
  EXPECT_THROW({ unbounded2->createSampleGenerator(); }, std::runtime_error);
}
