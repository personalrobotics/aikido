#include <gtest/gtest.h>

#include <aikido/common/RNG.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>

#include "eigen_tests.hpp"

using aikido::constraint::Satisfied;
using aikido::planner::parabolic::ParabolicSmoother;
using aikido::statespace::CartesianProduct;
using aikido::statespace::ConstStateSpacePtr;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R1;
using aikido::trajectory::Interpolated;
using Eigen::Vector2d;

class SmoothPostProcessorTests : public ::testing::Test
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  void SetUp() override
  {
    mRng = aikido::common::RNGWrapper<std::mt19937>(0);
    std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
    for (std::size_t i = 0; i < 2; ++i)
      subspaces.emplace_back(std::make_shared<R1>());
    mStateSpace = std::make_shared<CartesianProduct>(subspaces);
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
    mStateSpace->expMap(p1, state);
    mTrajectory->addWaypoint(0., state);
    mStateSpace->expMap(p2, state);
    mTrajectory->addWaypoint(1., state);
    mStateSpace->expMap(p3, state);
    mTrajectory->addWaypoint(2., state);
    mStateSpace->expMap(p4, state);
    mTrajectory->addWaypoint(3., state);
    mStateSpace->expMap(p5, state);
    mTrajectory->addWaypoint(4., state);
    mStateSpace->expMap(p6, state);
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

  void evaluate(
      aikido::trajectory::Trajectory* traj,
      double t,
      Eigen::VectorXd& positions)
  {
    // This utility function is just for convenience;
    // states should be reused when possible.
    auto state = traj->getStateSpace()->createState();
    traj->evaluate(t, state);
    traj->getStateSpace()->logMap(state, positions);
  }

  aikido::common::RNGWrapper<std::mt19937> mRng;
  std::shared_ptr<CartesianProduct> mStateSpace;
  Eigen::Vector2d mMaxVelocity;
  Eigen::Vector2d mMaxAcceleration;

  std::shared_ptr<GeodesicInterpolator> mInterpolator;
  std::shared_ptr<Interpolated> mTrajectory;

  double mOriginalTrajectoryLength;
  double mTimelimit = 60.0;
  double mBlendRadius = 0.5;
  double mBlendIterations = 100;
  double mFeasibilityCheckResolution = 1e-1;
  double mFeasibilityApproxTolerance = 1e-3;
  const double mTolerance = 1e-5;
};

TEST_F(SmoothPostProcessorTests, useShortcutting)
{
  std::shared_ptr<Satisfied> testable
      = std::make_shared<Satisfied>(mStateSpace);
  bool enableShortcut = true;
  bool enableBlend = false;

  ParabolicSmoother testSmoothPostProcessor(
      mMaxVelocity,
      mMaxAcceleration,
      enableShortcut,
      enableBlend,
      mTimelimit,
      mBlendRadius,
      mBlendIterations,
      mFeasibilityCheckResolution,
      mFeasibilityApproxTolerance);

  auto smoothedTrajectory
      = testSmoothPostProcessor.postprocess(*mTrajectory, mRng, testable);

  // Position.
  Eigen::VectorXd statePositions, startPositions, goalPositions;
  evaluate(mTrajectory.get(), mTrajectory->getStartTime(), startPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getStartTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(startPositions, statePositions, mTolerance);

  evaluate(mTrajectory.get(), mTrajectory->getEndTime(), goalPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getEndTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(goalPositions, statePositions, mTolerance);

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

  ParabolicSmoother testSmoothPostProcessor(
      mMaxVelocity,
      mMaxAcceleration,
      enableShortcut,
      enableBlend,
      mTimelimit,
      mBlendRadius,
      mBlendIterations,
      mFeasibilityCheckResolution,
      mFeasibilityApproxTolerance);

  auto smoothedTrajectory
      = testSmoothPostProcessor.postprocess(*mTrajectory, mRng, testable);

  // Position.
  Eigen::VectorXd statePositions, startPositions, goalPositions;
  evaluate(mTrajectory.get(), mTrajectory->getStartTime(), startPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getStartTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(startPositions, statePositions, mTolerance);

  evaluate(mTrajectory.get(), mTrajectory->getEndTime(), goalPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getEndTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(goalPositions, statePositions, mTolerance);

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

  ParabolicSmoother testSmoothPostProcessor(
      mMaxVelocity,
      mMaxAcceleration,
      enableShortcut,
      enableBlend,
      mTimelimit,
      mBlendRadius,
      mBlendIterations,
      mFeasibilityCheckResolution,
      mFeasibilityApproxTolerance);

  auto smoothedTrajectory
      = testSmoothPostProcessor.postprocess(*mTrajectory, mRng, testable);

  // Position.
  Eigen::VectorXd statePositions, startPositions, goalPositions;
  evaluate(mTrajectory.get(), mTrajectory->getStartTime(), startPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getStartTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(startPositions, statePositions, mTolerance);

  evaluate(mTrajectory.get(), mTrajectory->getEndTime(), goalPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getEndTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(goalPositions, statePositions, mTolerance);

  double shortenLength = getLength(smoothedTrajectory.get());
  EXPECT_TRUE(shortenLength < mOriginalTrajectoryLength);

  double originTime = mTrajectory->getDuration();
  double shortenTime = smoothedTrajectory->getDuration();
  EXPECT_TRUE(shortenTime < originTime);
}
