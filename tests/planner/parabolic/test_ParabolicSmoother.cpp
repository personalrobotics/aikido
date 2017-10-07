#include <gtest/gtest.h>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/SO3.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/common/StepSequence.hpp>
#include "eigen_tests.hpp"

using Eigen::Vector2d;
using Eigen::Vector3d;
using aikido::trajectory::Interpolated;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R2;
using aikido::statespace::CartesianProduct;
using aikido::statespace::SO2;
using aikido::statespace::SO3;
using aikido::statespace::StateSpacePtr;
using aikido::constraint::Satisfied;
using aikido::planner::parabolic::computeParabolicTiming;
using aikido::planner::parabolic::convertToSpline;
using aikido::planner::parabolic::doShortcut;
using aikido::planner::parabolic::doBlend;
using aikido::planner::parabolic::doShortcutAndBlend;

class ParabolicSmootherTests : public ::testing::Test
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  void SetUp() override
  {
    mRng = aikido::common::RNGWrapper<std::mt19937>( 0 );
    mStateSpace = std::make_shared<R2>();
    mMaxVelocity = Eigen::Vector2d(20., 20.);
    mMaxAcceleration = Eigen::Vector2d(10., 10.);


    mInterpolator = std::make_shared<GeodesicInterpolator>(mStateSpace);

    mStraightLineLength = initStraightLine();
    mNonStraightLineLength = initNonStraightLine();
    mNonStraightLineWithNonZeroStartTimeLength =
      initNonStraightLineWithNonZeroStartTime();
  }

  double initStraightLine()
  {
      mStraightLine = std::make_shared<Interpolated>(
        mStateSpace, mInterpolator);

      auto state = mStateSpace->createState();
      Vector2d p1(1., 2.), p2(3., 4.);
      state.setValue(p1);
      mStraightLine->addWaypoint(0., state);
      state.setValue(p2);
      mStraightLine->addWaypoint(1., state);

      return (p2-p1).norm();
  }

  double initNonStraightLine()
  {
    mNonStraightLine = std::make_shared<Interpolated>
            (mStateSpace, mInterpolator);

    auto state = mStateSpace->createState();
    Vector2d p1(1., 1.), p2(2., 1.2), p3(2.2, 2.),
            p4(3., 2.2), p5(3.2, 3.), p6(4.,3.2);
    state.setValue(p1);
    mNonStraightLine->addWaypoint(0., state);
    state.setValue(p2);
    mNonStraightLine->addWaypoint(1., state);
    state.setValue(p3);
    mNonStraightLine->addWaypoint(2., state);
    state.setValue(p4);
    mNonStraightLine->addWaypoint(3., state);
    state.setValue(p5);
    mNonStraightLine->addWaypoint(4., state);
    state.setValue(p6);
    mNonStraightLine->addWaypoint(5, state);

    double length = (p2-p1).norm() + (p3-p2).norm()
            + (p4-p3).norm() + (p5-p4).norm()
            + (p6-p5).norm();
    return length;
  }

  double initNonStraightLineWithNonZeroStartTime()
  {
    mNonStraightLineWithNonZeroStartTime = std::make_shared<Interpolated>
            (mStateSpace, mInterpolator);

    auto state = mStateSpace->createState();
    Vector2d p1(1., 1.), p2(2., 1.2), p3(2.2, 2.),
            p4(3., 2.2), p5(3.2, 3.);
    state.setValue(p1);
    mNonStraightLineWithNonZeroStartTime->addWaypoint(1.2, state);
    state.setValue(p2);
    mNonStraightLineWithNonZeroStartTime->addWaypoint(2.3, state);
    state.setValue(p3);
    mNonStraightLineWithNonZeroStartTime->addWaypoint(3.4, state);
    state.setValue(p4);
    mNonStraightLineWithNonZeroStartTime->addWaypoint(4.5, state);
    state.setValue(p5);
    mNonStraightLineWithNonZeroStartTime->addWaypoint(5.5, state);

    double length = (p2-p1).norm() + (p3-p2).norm()
            + (p4-p3).norm() + (p5-p4).norm();
    return length;
  }

  double getLength(aikido::trajectory::Spline* spline)
  {
    double length = 0.0;
    auto currState = mStateSpace->createState();
    auto nextState = mStateSpace->createState();
    Eigen::VectorXd currVec, nextVec;
    for(std::size_t i=1; i<spline->getNumWaypoints();++i)
    {
      spline->getWaypoint(i-1, currState);
      spline->getWaypoint(i, nextState);
      mStateSpace->logMap(currState, currVec);
      mStateSpace->logMap(nextState, nextVec);
      length += (nextVec-currVec).norm();
    }
    return length;
  }

  aikido::common::RNGWrapper<std::mt19937> mRng;
  std::shared_ptr<R2> mStateSpace;
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
  const double mTolerance = 1e-5;
};

TEST_F(ParabolicSmootherTests, convertStraightInterpolatedToSpline)
{
    auto spline = convertToSpline(*mStraightLine);

    auto splineState = mStateSpace->createState();
    auto interpolatedState = mStateSpace->createState();
    Eigen::VectorXd splineVec, interpolatedVec;
    Eigen::VectorXd splineTangent, interpolatedTangent;

    const double stepSize = 1e-3;
    aikido::common::StepSequence seq(stepSize, true,
                                   spline->getStartTime(),
                                   spline->getEndTime());
    for(double t : seq)
    {
      spline->evaluate(t, splineState);
      mStraightLine->evaluate(t, interpolatedState);
      mStateSpace->logMap(splineState, splineVec);
      mStateSpace->logMap(interpolatedState, interpolatedVec);
      EXPECT_EIGEN_EQUAL(splineVec, interpolatedVec, mTolerance);
    }
}

TEST_F(ParabolicSmootherTests, convertNonStraightInterpolatedToSpline)
{
  auto spline = convertToSpline(*mNonStraightLine);

  auto splineState = mStateSpace->createState();
  auto interpolatedState = mStateSpace->createState();
  Eigen::VectorXd splineVec, interpolatedVec;
  Eigen::VectorXd splineTangent, interpolatedTangent;

  const double stepSize = 1e-3;
  aikido::common::StepSequence seq(stepSize, true,
                                 spline->getStartTime(),
                                 spline->getEndTime());
  for(double t : seq)
  {
    spline->evaluate(t, splineState);
    mNonStraightLine->evaluate(t, interpolatedState);
    mStateSpace->logMap(splineState, splineVec);
    mStateSpace->logMap(interpolatedState, interpolatedVec);
    EXPECT_EIGEN_EQUAL(splineVec, interpolatedVec, mTolerance);
  }
}

TEST_F(ParabolicSmootherTests, convertNonStraightInterolatedWithNonZeroStartTimeToSpline)
{
    auto spline = convertToSpline(*mNonStraightLineWithNonZeroStartTime);

    auto splineState = mStateSpace->createState();
    auto interpolatedState = mStateSpace->createState();
    Eigen::VectorXd splineVec, interpolatedVec;
    Eigen::VectorXd splineTangent, interpolatedTangent;

    const double stepSize = 1e-3;
    aikido::common::StepSequence seq(stepSize, true,
                                   spline->getStartTime(),
                                   spline->getEndTime());
    for(double t : seq)
    {
      spline->evaluate(t, splineState);
      mNonStraightLineWithNonZeroStartTime->evaluate(t, interpolatedState);
      mStateSpace->logMap(splineState, splineVec);
      mStateSpace->logMap(interpolatedState, interpolatedVec);
      EXPECT_EIGEN_EQUAL(splineVec, interpolatedVec, mTolerance);
    }
}

TEST_F(ParabolicSmootherTests, doShortcut)
{
   std::shared_ptr<Satisfied> testable =
           std::make_shared<Satisfied>(mStateSpace);

   auto splineTrajectory = computeParabolicTiming(*mNonStraightLine,
       mMaxVelocity, mMaxAcceleration);
   auto smoothedTrajectory = doShortcut(*splineTrajectory.get(),
       testable, mMaxVelocity, mMaxAcceleration, mRng, mTimelimit);

   // Position.
   auto state = mStateSpace->createState();
   smoothedTrajectory->evaluate(smoothedTrajectory->getStartTime(), state);
   auto startState = mStateSpace->createState();
   mNonStraightLine->evaluate(mNonStraightLine->getStartTime(), startState);
   EXPECT_EIGEN_EQUAL(startState.getValue(), state.getValue(), mTolerance);

   smoothedTrajectory->evaluate(smoothedTrajectory->getEndTime(), state);
   auto goalState = mStateSpace->createState();
   mNonStraightLine->evaluate(mNonStraightLine->getEndTime(), goalState);
   EXPECT_EIGEN_EQUAL(goalState.getValue(), state.getValue(), mTolerance);

   double shortenLength = getLength(smoothedTrajectory.get());
   EXPECT_TRUE(shortenLength < mNonStraightLineLength);

   double originTime = mNonStraightLine->getDuration();
   double shortenTime = smoothedTrajectory->getDuration();
   EXPECT_TRUE(shortenTime<originTime);
}

TEST_F(ParabolicSmootherTests, doBlend)
{
   std::shared_ptr<Satisfied> testable =
           std::make_shared<Satisfied>(mStateSpace);

   auto splineTrajectory = computeParabolicTiming(*mNonStraightLine,
       mMaxVelocity, mMaxAcceleration);
   double blendRadius = 0.5;
   int blendIterations = 100;
   auto smoothedTrajectory = doBlend(
               *splineTrajectory.get(), testable,
               mMaxVelocity, mMaxAcceleration, blendRadius, blendIterations);

   // Position.
   auto state = mStateSpace->createState();
   smoothedTrajectory->evaluate(smoothedTrajectory->getStartTime(), state);
   auto startState = mStateSpace->createState();
   mNonStraightLine->evaluate(mNonStraightLine->getStartTime(), startState);
   EXPECT_EIGEN_EQUAL(startState.getValue(), state.getValue(), mTolerance);

   smoothedTrajectory->evaluate(smoothedTrajectory->getEndTime(), state);
   auto goalState = mStateSpace->createState();
   mNonStraightLine->evaluate(mNonStraightLine->getEndTime(), goalState);
   EXPECT_EIGEN_EQUAL(goalState.getValue(), state.getValue(), mTolerance);

   double shortenLength = getLength(smoothedTrajectory.get());
   EXPECT_TRUE(shortenLength < mNonStraightLineLength);

   double originTime = mNonStraightLine->getDuration();
   double shortenTime = smoothedTrajectory->getDuration();
   EXPECT_TRUE(shortenTime<originTime);
}

TEST_F(ParabolicSmootherTests, doShortcutAndBlend)
{
    std::shared_ptr<Satisfied> testable =
           std::make_shared<Satisfied>(mStateSpace);

   auto splineTrajectory = computeParabolicTiming(*mNonStraightLine,
       mMaxVelocity, mMaxAcceleration);
   double blendRadius = 0.5;
   int blendIterations = 100;
   auto smoothedTrajectory = doShortcutAndBlend(*splineTrajectory.get(),
       testable, mMaxVelocity, mMaxAcceleration, mRng, mTimelimit,
       blendRadius, blendIterations);

   // Position.
   auto state = mStateSpace->createState();
   smoothedTrajectory->evaluate(smoothedTrajectory->getStartTime(), state);
   auto startState = mStateSpace->createState();
   mNonStraightLine->evaluate(mNonStraightLine->getStartTime(), startState);
   EXPECT_EIGEN_EQUAL(startState.getValue(), state.getValue(), mTolerance);

   smoothedTrajectory->evaluate(smoothedTrajectory->getEndTime(), state);
   auto goalState = mStateSpace->createState();
   mNonStraightLine->evaluate(mNonStraightLine->getEndTime(), goalState);
   EXPECT_EIGEN_EQUAL(goalState.getValue(), state.getValue(), mTolerance);

   double shortenLength = getLength(smoothedTrajectory.get());
   EXPECT_TRUE(shortenLength < mNonStraightLineLength);

   double originTime = mNonStraightLine->getDuration();
   double shortenTime = smoothedTrajectory->getDuration();
   EXPECT_TRUE(shortenTime<originTime);
}
