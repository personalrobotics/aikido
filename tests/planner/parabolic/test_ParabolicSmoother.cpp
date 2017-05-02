#include <gtest/gtest.h>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/SO3.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/util/StepSequence.hpp>
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
    mStateSpace = std::make_shared<R2>();
    mMaxVelocity = Eigen::Vector2d(1., 1.);
    mMaxAcceleration = Eigen::Vector2d(2., 2.);

    mInterpolator = std::make_shared<GeodesicInterpolator>(mStateSpace);
    mStraightLine = std::make_shared<Interpolated>(
      mStateSpace, mInterpolator);

    auto state = mStateSpace->createState();

    state.setValue(Vector2d(1., 2.));
    mStraightLine->addWaypoint(0., state);

    state.setValue(Vector2d(3., 4.));
    mStraightLine->addWaypoint(1., state);

    mNonStraightLineLength = initNonStraightLine();
  }

  double initNonStraightLine()
  {
    mNonStraightLine = std::make_shared<Interpolated>
            (mStateSpace, mInterpolator);

    auto state = mStateSpace->createState();
    Vector2d p1(1., 1.), p2(2., 1.), p3(2., 2.),
            p4(3., 2.), p5(3., 3.), p6(4.,3.);
    state.setValue(p1);
    mNonStraightLine->addWaypoint(0., state);
    state.setValue(p2);
    mNonStraightLine->addWaypoint(0.25, state);
    state.setValue(p3);
    mNonStraightLine->addWaypoint(0.5, state);
    state.setValue(p4);
    mNonStraightLine->addWaypoint(0.75, state);
    state.setValue(p5);
    mNonStraightLine->addWaypoint(1., state);
    state.setValue(p6);
    mNonStraightLine->addWaypoint(1.25, state);

    double length = (p2-p1).norm() + (p3-p2).norm()
            + (p4-p3).norm() + (p5-p4).norm()
            + (p6-p5).norm();
    return length;
  }

  double getLength(aikido::trajectory::Spline* spline)
  {
    double length = 0.0;
    auto currState = mStateSpace->createState();
    auto nextState = mStateSpace->createState();
    Eigen::VectorXd currVec, nextVec;
    for(size_t i=0; i<spline->getNumWaypoints()-1;++i)
    {
      spline->getWaypoint(i, currState);
      spline->getWaypoint(i+1, nextState);
      mStateSpace->logMap(currState, currVec);
      mStateSpace->logMap(nextState, nextVec);
      length += (nextVec-currVec).norm();
    }
    return length;
  }

  std::shared_ptr<R2> mStateSpace;
  Eigen::Vector2d mMaxVelocity;
  Eigen::Vector2d mMaxAcceleration;

  std::shared_ptr<GeodesicInterpolator> mInterpolator;
  std::shared_ptr<Interpolated> mStraightLine;
  std::shared_ptr<Interpolated> mNonStraightLine;
  double mNonStraightLineLength;
  const double mTolerance = 1e-5;
};

TEST_F(ParabolicSmootherTests, convertInterpolatedToSpline)
{

    auto spline = convertToSpline(*mStraightLine.get());

    auto splineState = mStateSpace->createState();
    auto interpolatedState = mStateSpace->createState();
    Eigen::VectorXd splineVec, interpolatedVec;
    Eigen::VectorXd splineTangent, interpolatedTangent;

    const double stepSize = 1e-3;
    aikido::util::StepSequence seq(stepSize, true,
                                   spline->getStartTime(),
                                   spline->getEndTime());
    for(double t : seq)
    {
        spline->evaluate(t, splineState);
        mStraightLine->evaluate(t, interpolatedState);
        mStateSpace->logMap(splineState, splineVec);
        mStateSpace->logMap(interpolatedState, interpolatedVec);
        EXPECT_EIGEN_EQUAL(splineVec, interpolatedVec, mTolerance);

        spline->evaluateDerivative(t, 1, splineTangent);
        mStraightLine->evaluateDerivative(t, 1, interpolatedTangent);
        EXPECT_EIGEN_EQUAL(splineTangent, interpolatedTangent, mTolerance);
    }
}

TEST_F(ParabolicSmootherTests, doShortcut)
{
   std::shared_ptr<Satisfied> testable =
           std::make_shared<Satisfied>(mStateSpace);

   auto splineTrajectory = convertToSpline(*mNonStraightLine.get());
   double timelimit = 10.0;
   auto smoothedTrajectory = doShortcut(*splineTrajectory.get(),
       testable, mMaxVelocity, mMaxAcceleration, timelimit);

   double shortenLength = getLength(smoothedTrajectory.get());

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

   EXPECT_TRUE(shortenLength < mNonStraightLineLength);
}

TEST_F(ParabolicSmootherTests, doBlend)
{
   Eigen::Vector2d maxVelocity(10., 10.), maxAcceleration(20., 20.);
   std::shared_ptr<Satisfied> testable =
           std::make_shared<Satisfied>(mStateSpace);

   auto splineTrajectory = convertToSpline(*mNonStraightLine.get());
   double timelimit = 10.0;
   double blendRadius = 0.5;
   int blendIterations = 10;
   auto smoothedTrajectory = doBlend(*splineTrajectory.get(),
       testable, maxVelocity, maxAcceleration, timelimit, blendRadius, blendIterations);

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

   double originTime = mNonStraightLine->getDuration();
   double shortenTime = smoothedTrajectory->getDuration();
   EXPECT_TRUE(shortenTime<originTime);
}

TEST_F(ParabolicSmootherTests, doShortcutAndBlend)
{
   Eigen::Vector2d maxVelocity(10., 10.), maxAcceleration(20., 20.);

    std::shared_ptr<Satisfied> testable =
           std::make_shared<Satisfied>(mStateSpace);

   auto splineTrajectory = convertToSpline(*mNonStraightLine.get());
   double timelimit = 10.0;
   double blendRadius = 0.5;
   int blendIterations = 10;
   auto smoothedTrajectory = doShortcutAndBlend(*splineTrajectory.get(),
       testable, maxVelocity, maxAcceleration, timelimit,
       blendRadius, blendIterations);

   double shortenLength = getLength(smoothedTrajectory.get());

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

   EXPECT_TRUE(shortenLength < mNonStraightLineLength);

   double originTime = mNonStraightLine->getDuration();
   double shortenTime = smoothedTrajectory->getDuration();
   EXPECT_TRUE(shortenTime<originTime);
}
