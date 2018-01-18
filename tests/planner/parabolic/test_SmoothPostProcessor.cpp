#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include "eigen_tests.hpp"

using Eigen::Vector2d;
using Eigen::Vector3d;
using aikido::trajectory::Interpolated;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R2;
using aikido::statespace::StateSpacePtr;
using aikido::constraint::Satisfied;
using aikido::common::cloneRNGFrom;
using aikido::planner::parabolic::SmoothTrajectoryPostProcessor;

class SmoothPostProcessorTests : public ::testing::Test
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  void SetUp() override
  {
    mRng = aikido::common::RNGWrapper<std::mt19937>(0);
    mStateSpace = std::make_shared<R2>();
    mMaxVelocity = Eigen::Vector2d(20., 20.);
    mMaxAcceleration = Eigen::Vector2d(10., 10.);

    mInterpolator = std::make_shared<GeodesicInterpolator>(mStateSpace);
    mOriginalTrajectoryLength = initNonStraightLine();
  }

  double initNonStraightLine()
  {
    mTrajectory = std::make_shared<Interpolated>(mStateSpace, mInterpolator);

    auto state = mStateSpace->createState();
    Vector2d p1(1., 1.), p2(2., 1.2), p3(2.2, 2.), p4(3., 2.2), p5(3.2, 3.),
        p6(4., 3.2);
    state.setValue(p1);
    mTrajectory->addWaypoint(0., state);
    state.setValue(p2);
    mTrajectory->addWaypoint(1., state);
    state.setValue(p3);
    mTrajectory->addWaypoint(2., state);
    state.setValue(p4);
    mTrajectory->addWaypoint(3., state);
    state.setValue(p5);
    mTrajectory->addWaypoint(4., state);
    state.setValue(p6);
    mTrajectory->addWaypoint(5, state);

    double length = (p2 - p1).norm() + (p3 - p2).norm() + (p4 - p3).norm()
                    + (p5 - p4).norm() + (p6 - p5).norm();
    return length;
  }

  double getLength(aikido::trajectory::Spline* spline)
  {
    double length = 0.0;
    auto currState = mStateSpace->createState();
    auto nextState = mStateSpace->createState();
    Eigen::VectorXd currVec, nextVec;
    for (std::size_t i = 1; i < spline->getNumWaypoints(); ++i)
    {
      spline->getWaypoint(i - 1, currState);
      spline->getWaypoint(i, nextState);
      mStateSpace->logMap(currState, currVec);
      mStateSpace->logMap(nextState, nextVec);
      length += (nextVec - currVec).norm();
    }
    return length;
  }

  aikido::common::RNGWrapper<std::mt19937> mRng;
  std::shared_ptr<R2> mStateSpace;
  Eigen::Vector2d mMaxVelocity;
  Eigen::Vector2d mMaxAcceleration;

  std::shared_ptr<GeodesicInterpolator> mInterpolator;
  std::shared_ptr<Interpolated> mTrajectory;

  double mOriginalTrajectoryLength;
  double mFeasibilityCheckResolution = 1e-4;
  double mFeasibilityApproxTolerance = 1e-3;
  double mBlendRadius = 0.5;
  double mBlendIterations = 100;
  double mTimelimit = 60.0;
  const double mTolerance = 1e-5;
};

TEST_F(SmoothPostProcessorTests, useShortcutting)
{
  std::shared_ptr<Satisfied> testable
      = std::make_shared<Satisfied>(mStateSpace);
  bool enableShortcut = true;
  bool enableBlend = false;

  SmoothTrajectoryPostProcessor testSmoothPostProcessor(
      mMaxVelocity,
      mMaxAcceleration,
      testable,
      enableShortcut,
      enableBlend,
      mTimelimit,
      mBlendRadius,
      mBlendIterations,
      mFeasibilityCheckResolution,
      mFeasibilityApproxTolerance);

  auto clonedRNG = std::move(cloneRNGFrom(mRng)[0]);
  auto smoothedTrajectory
      = testSmoothPostProcessor.postprocess(mTrajectory, clonedRNG.get());

  // Position.
  auto state = mStateSpace->createState();
  smoothedTrajectory->evaluate(smoothedTrajectory->getStartTime(), state);
  auto startState = mStateSpace->createState();
  mTrajectory->evaluate(mTrajectory->getStartTime(), startState);
  EXPECT_EIGEN_EQUAL(startState.getValue(), state.getValue(), mTolerance);

  smoothedTrajectory->evaluate(smoothedTrajectory->getEndTime(), state);
  auto goalState = mStateSpace->createState();
  mTrajectory->evaluate(mTrajectory->getEndTime(), goalState);
  EXPECT_EIGEN_EQUAL(goalState.getValue(), state.getValue(), mTolerance);

  double shortenLength = getLength(smoothedTrajectory.get());
  EXPECT_TRUE(shortenLength < mOriginalTrajectoryLength);

  double originTime = mTrajectory->getDuration();
  double shortenTime = smoothedTrajectory->getDuration();
  EXPECT_TRUE(shortenTime < originTime);
}

TEST_F(SmoothPostProcessorTests, useBlend)
{
  std::shared_ptr<Satisfied> testable
      = std::make_shared<Satisfied>(mStateSpace);
  bool enableShortcut = false;
  bool enableBlend = true;

  SmoothTrajectoryPostProcessor testSmoothPostProcessor(
      mMaxVelocity,
      mMaxAcceleration,
      testable,
      enableShortcut,
      enableBlend,
      mTimelimit,
      mBlendRadius,
      mBlendIterations,
      mFeasibilityCheckResolution,
      mFeasibilityApproxTolerance);

  auto clonedRNG = std::move(cloneRNGFrom(mRng)[0]);
  auto smoothedTrajectory
      = testSmoothPostProcessor.postprocess(mTrajectory, clonedRNG.get());

  // Position.
  auto state = mStateSpace->createState();
  smoothedTrajectory->evaluate(smoothedTrajectory->getStartTime(), state);
  auto startState = mStateSpace->createState();
  mTrajectory->evaluate(mTrajectory->getStartTime(), startState);
  EXPECT_EIGEN_EQUAL(startState.getValue(), state.getValue(), mTolerance);

  smoothedTrajectory->evaluate(smoothedTrajectory->getEndTime(), state);
  auto goalState = mStateSpace->createState();
  mTrajectory->evaluate(mTrajectory->getEndTime(), goalState);
  EXPECT_EIGEN_EQUAL(goalState.getValue(), state.getValue(), mTolerance);

  double shortenLength = getLength(smoothedTrajectory.get());
  EXPECT_TRUE(shortenLength < mOriginalTrajectoryLength);

  double originTime = mTrajectory->getDuration();
  double shortenTime = smoothedTrajectory->getDuration();
  EXPECT_TRUE(shortenTime < originTime);
}

TEST_F(SmoothPostProcessorTests, useShortcuttingAndBlend)
{
  std::shared_ptr<Satisfied> testable
      = std::make_shared<Satisfied>(mStateSpace);
  bool enableShortcut = true;
  bool enableBlend = true;

  SmoothTrajectoryPostProcessor testSmoothPostProcessor(
      mMaxVelocity,
      mMaxAcceleration,
      testable,
      enableShortcut,
      enableBlend,
      mTimelimit,
      mBlendRadius,
      mBlendIterations,
      mFeasibilityCheckResolution,
      mFeasibilityApproxTolerance);

  auto clonedRNG = std::move(cloneRNGFrom(mRng)[0]);
  auto smoothedTrajectory
      = testSmoothPostProcessor.postprocess(mTrajectory, clonedRNG.get());

  // Position.
  auto state = mStateSpace->createState();
  smoothedTrajectory->evaluate(smoothedTrajectory->getStartTime(), state);
  auto startState = mStateSpace->createState();
  mTrajectory->evaluate(mTrajectory->getStartTime(), startState);
  EXPECT_EIGEN_EQUAL(startState.getValue(), state.getValue(), mTolerance);

  smoothedTrajectory->evaluate(smoothedTrajectory->getEndTime(), state);
  auto goalState = mStateSpace->createState();
  mTrajectory->evaluate(mTrajectory->getEndTime(), goalState);
  EXPECT_EIGEN_EQUAL(goalState.getValue(), state.getValue(), mTolerance);

  double shortenLength = getLength(smoothedTrajectory.get());
  EXPECT_TRUE(shortenLength < mOriginalTrajectoryLength);

  double originTime = mTrajectory->getDuration();
  double shortenTime = smoothedTrajectory->getDuration();
  EXPECT_TRUE(shortenTime < originTime);
}
