#include <gtest/gtest.h>
#include <aikido/common/StepSequence.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/trajectory/util.hpp>
#include "eigen_tests.hpp"

using aikido::constraint::Satisfied;
using aikido::planner::parabolic::computeParabolicTiming;
using aikido::planner::parabolic::doBlend;
using aikido::planner::parabolic::doShortcut;
using aikido::planner::parabolic::doShortcutAndBlend;
using aikido::statespace::CartesianProduct;
using aikido::statespace::ConstStateSpacePtr;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R1;
using aikido::trajectory::convertToSpline;
using aikido::trajectory::Interpolated;
using Eigen::Vector2d;

class ParabolicSmootherTests : public ::testing::Test
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

    mStraightLineLength = initStraightLine();
    mNonStraightLineLength = initNonStraightLine();
    mNonStraightLineWithNonZeroStartTimeLength
        = initNonStraightLineWithNonZeroStartTime();
  }

  double initStraightLine()
  {
    mStraightLine = std::make_shared<Interpolated>(mStateSpace, mInterpolator);

    auto state = mStateSpace->createState();
    Vector2d p1(1, 2), p2(3, 4);
    mStateSpace->expMap(p1, state);
    mStraightLine->addWaypoint(0., state);
    mStateSpace->expMap(p2, state);
    mStraightLine->addWaypoint(1., state);

    return (p2 - p1).norm();
  }

  double initNonStraightLine()
  {
    mNonStraightLine
        = std::make_shared<Interpolated>(mStateSpace, mInterpolator);

    auto state = mStateSpace->createState();

    Vector2d p1(1., 1.), p2(2., 1.2), p3(2.2, 2.), p4(3., 2.2), p5(3.2, 3.),
        p6(4., 3.2);
    mStateSpace->expMap(p1, state);
    mNonStraightLine->addWaypoint(0., state);
    mStateSpace->expMap(p2, state);
    mNonStraightLine->addWaypoint(1., state);
    mStateSpace->expMap(p3, state);
    mNonStraightLine->addWaypoint(2., state);
    mStateSpace->expMap(p4, state);
    mNonStraightLine->addWaypoint(3., state);
    mStateSpace->expMap(p5, state);
    mNonStraightLine->addWaypoint(4., state);
    mStateSpace->expMap(p6, state);
    mNonStraightLine->addWaypoint(5, state);

    double length = (p2 - p1).norm() + (p3 - p2).norm() + (p4 - p3).norm()
                    + (p5 - p4).norm() + (p6 - p5).norm();
    return length;
  }

  double initNonStraightLineWithNonZeroStartTime()
  {
    mNonStraightLineWithNonZeroStartTime
        = std::make_shared<Interpolated>(mStateSpace, mInterpolator);

    auto state = mStateSpace->createState();
    Vector2d p1(1., 1.), p2(2., 1.2), p3(2.2, 2.), p4(3., 2.2), p5(3.2, 3.);
    mStateSpace->expMap(p1, state);
    mNonStraightLineWithNonZeroStartTime->addWaypoint(1.2, state);
    mStateSpace->expMap(p2, state);
    mNonStraightLineWithNonZeroStartTime->addWaypoint(2.3, state);
    mStateSpace->expMap(p3, state);
    mNonStraightLineWithNonZeroStartTime->addWaypoint(3.4, state);
    mStateSpace->expMap(p4, state);
    mNonStraightLineWithNonZeroStartTime->addWaypoint(4.5, state);
    mStateSpace->expMap(p5, state);
    mNonStraightLineWithNonZeroStartTime->addWaypoint(5.5, state);

    double length = (p2 - p1).norm() + (p3 - p2).norm() + (p4 - p3).norm()
                    + (p5 - p4).norm();
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
  std::shared_ptr<Interpolated> mStraightLine;
  std::shared_ptr<Interpolated> mNonStraightLine;
  std::shared_ptr<Interpolated> mNonStraightLineWithNonZeroStartTime;

  double mStraightLineLength;
  double mNonStraightLineLength;
  double mNonStraightLineWithNonZeroStartTimeLength;
  double mTimelimit = 60.0;
  double mBlendRadius = 0.5;
  int mBlendIterations = 4;
  double mCheckResolution = 1e-1;
  const double mTolerance = 1e-5;
};

TEST_F(ParabolicSmootherTests, convertStraightInterpolatedToSpline)
{
  auto spline = convertToSpline(*mStraightLine);

  Eigen::VectorXd splineVec, interpolatedVec;

  const double stepSize = 1e-3;
  aikido::common::StepSequence seq(
      stepSize, true, true, spline->getStartTime(), spline->getEndTime());
  for (double t : seq)
  {
    evaluate(spline.get(), t, splineVec);
    evaluate(mStraightLine.get(), t, interpolatedVec);
    EXPECT_EIGEN_EQUAL(splineVec, interpolatedVec, mTolerance);
  }
}

TEST_F(ParabolicSmootherTests, convertNonStraightInterpolatedToSpline)
{
  auto spline = convertToSpline(*mNonStraightLine);

  Eigen::VectorXd splineVec, interpolatedVec;

  const double stepSize = 1e-3;
  aikido::common::StepSequence seq(
      stepSize, true, true, spline->getStartTime(), spline->getEndTime());
  for (double t : seq)
  {
    evaluate(spline.get(), t, splineVec);
    evaluate(mNonStraightLine.get(), t, interpolatedVec);
    EXPECT_EIGEN_EQUAL(splineVec, interpolatedVec, mTolerance);
  }
}

TEST_F(
    ParabolicSmootherTests,
    convertNonStraightInterpolatedWithNonZeroStartTimeToSpline)
{
  auto spline = convertToSpline(*mNonStraightLineWithNonZeroStartTime);

  Eigen::VectorXd splineVec, interpolatedVec;

  const double stepSize = 1e-3;
  aikido::common::StepSequence seq(
      stepSize, true, true, spline->getStartTime(), spline->getEndTime());
  for (double t : seq)
  {
    evaluate(spline.get(), t, splineVec);
    evaluate(mNonStraightLineWithNonZeroStartTime.get(), t, interpolatedVec);
    EXPECT_EIGEN_EQUAL(splineVec, interpolatedVec, mTolerance);
  }
}

TEST_F(ParabolicSmootherTests, doShortcut)
{
  std::shared_ptr<Satisfied> testable
      = std::make_shared<Satisfied>(mStateSpace);

  auto splineTrajectory = computeParabolicTiming(
      *mNonStraightLine, mMaxVelocity, mMaxAcceleration);
  auto smoothedTrajectory = doShortcut(
      *splineTrajectory.get(),
      testable,
      mMaxVelocity,
      mMaxAcceleration,
      mRng,
      mTimelimit,
      mCheckResolution);

  // Position.
  Eigen::VectorXd statePositions, startPositions, goalPositions;

  evaluate(
      mNonStraightLine.get(), mNonStraightLine->getStartTime(), startPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getStartTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(startPositions, statePositions, mTolerance);

  evaluate(
      mNonStraightLine.get(), mNonStraightLine->getEndTime(), goalPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getEndTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(goalPositions, statePositions, mTolerance);

  double shortenLength = getLength(smoothedTrajectory.get());
  EXPECT_TRUE(shortenLength < mNonStraightLineLength);

  double originTime = mNonStraightLine->getDuration();
  double shortenTime = smoothedTrajectory->getDuration();
  EXPECT_TRUE(shortenTime < originTime);
}

TEST_F(ParabolicSmootherTests, doBlend)
{
  std::shared_ptr<Satisfied> testable
      = std::make_shared<Satisfied>(mStateSpace);

  auto splineTrajectory = computeParabolicTiming(
      *mNonStraightLine, mMaxVelocity, mMaxAcceleration);
  auto smoothedTrajectory = doBlend(
      *splineTrajectory.get(),
      testable,
      mMaxVelocity,
      mMaxAcceleration,
      mBlendRadius,
      mBlendIterations,
      mCheckResolution);

  // Position.
  Eigen::VectorXd statePositions, startPositions, goalPositions;
  evaluate(
      mNonStraightLine.get(), mNonStraightLine->getStartTime(), startPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getStartTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(startPositions, statePositions, mTolerance);

  evaluate(
      mNonStraightLine.get(), mNonStraightLine->getEndTime(), goalPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getEndTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(goalPositions, statePositions, mTolerance);

  double shortenLength = getLength(smoothedTrajectory.get());
  EXPECT_TRUE(shortenLength < mNonStraightLineLength);

  double originTime = mNonStraightLine->getDuration();
  double shortenTime = smoothedTrajectory->getDuration();
  EXPECT_TRUE(shortenTime < originTime);
}

TEST_F(ParabolicSmootherTests, doShortcutAndBlend)
{
  std::shared_ptr<Satisfied> testable
      = std::make_shared<Satisfied>(mStateSpace);

  auto splineTrajectory = computeParabolicTiming(
      *mNonStraightLine, mMaxVelocity, mMaxAcceleration);
  auto smoothedTrajectory = doShortcutAndBlend(
      *splineTrajectory.get(),
      testable,
      mMaxVelocity,
      mMaxAcceleration,
      mRng,
      mTimelimit,
      mBlendRadius,
      mBlendIterations,
      mCheckResolution);

  // Position.
  Eigen::VectorXd statePositions, startPositions, goalPositions;
  evaluate(
      mNonStraightLine.get(), mNonStraightLine->getStartTime(), startPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getStartTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(startPositions, statePositions, mTolerance);

  evaluate(
      mNonStraightLine.get(), mNonStraightLine->getEndTime(), goalPositions);
  evaluate(
      smoothedTrajectory.get(),
      smoothedTrajectory->getEndTime(),
      statePositions);
  EXPECT_EIGEN_EQUAL(goalPositions, statePositions, mTolerance);

  double shortenLength = getLength(smoothedTrajectory.get());
  EXPECT_TRUE(shortenLength < mNonStraightLineLength);

  double originTime = mNonStraightLine->getDuration();
  double shortenTime = smoothedTrajectory->getDuration();
  EXPECT_TRUE(shortenTime < originTime);
}
