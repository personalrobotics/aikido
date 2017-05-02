#include <gtest/gtest.h>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/SO3.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/constraint/Satisfied.hpp>

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

  }

  std::shared_ptr<R2> mStateSpace;
  Eigen::Vector2d mMaxVelocity;
  Eigen::Vector2d mMaxAcceleration;

  std::shared_ptr<GeodesicInterpolator> mInterpolator;
  std::shared_ptr<Interpolated> mStraightLine;
};

TEST_F(ParabolicSmootherTests, convertInterpolatedToSpline)
{
    auto spline = convertToSpline(*mStraightLine.get());

    auto splineState = mStateSpace->createState();
    auto interpolatedState = mStateSpace->createState();
    Eigen::VectorXd splineVec, interpolatedVec;
    Eigen::VectorXd splineTangent, interpolatedTangent;
    for(double t=spline->getStartTime();t<=spline->getEndTime();t+=1e-2)
    {
        spline->evaluate(t, splineState);
        mStraightLine->evaluate(t, interpolatedState);
        mStateSpace->logMap(splineState, splineVec);
        mStateSpace->logMap(interpolatedState, interpolatedVec);
        EXPECT_TRUE(interpolatedVec.isApprox(interpolatedVec));

        spline->evaluateDerivative(t, 1, splineTangent);
        mStraightLine->evaluateDerivative(t, 1, interpolatedTangent);
        EXPECT_TRUE(splineTangent.isApprox(interpolatedTangent));
    }
}

TEST_F(ParabolicSmootherTests, doShort)
{
   Interpolated nonStraightLine(mStateSpace, mInterpolator);

   auto state = mStateSpace->createState();
   Vector2d p1(1., 1.), p2(1., 2.), p3(2., 2.),
           p4(2., 3.), p5(3., 3.);
   state.setValue(p1);
   nonStraightLine.addWaypoint(0., state);
   state.setValue(p2);
   nonStraightLine.addWaypoint(0.25, state);
   state.setValue(p3);
   nonStraightLine.addWaypoint(0.5, state);
   state.setValue(p4);
   nonStraightLine.addWaypoint(0.75, state);
   state.setValue(p5);
   nonStraightLine.addWaypoint(1., state);

   double orignDist = (p2-p1).norm() + (p3-p2).norm()
           + (p4-p3).norm() + (p5-p4).norm();

   std::shared_ptr<Satisfied> testable =
           std::make_shared<Satisfied>(mStateSpace);

   auto smoothedTrajectory = doShortcut(
       nonStraightLine,
       testable,
       mMaxVelocity,
       mMaxAcceleration);

   double shortenDist = 0.0;
   auto currState = mStateSpace->createState();
   auto nextState = mStateSpace->createState();
   Eigen::VectorXd currVec, nextVec;
   for(size_t i=0; i<smoothedTrajectory->getNumWaypoints()-1;++i)
   {
       smoothedTrajectory->getWaypoint(i, currState);
       smoothedTrajectory->getWaypoint(i+1, nextState);
       mStateSpace->logMap(currState, currVec);
       mStateSpace->logMap(nextState, nextVec);
       shortenDist += (nextVec-currVec).norm();
   }

   // Position.
   smoothedTrajectory->evaluate(smoothedTrajectory->getStartTime(), state);
   EXPECT_TRUE(p1.isApprox(state.getValue()));

   smoothedTrajectory->evaluate(smoothedTrajectory->getEndTime(), state);
   EXPECT_TRUE(p5.isApprox(state.getValue()));

   EXPECT_TRUE(shortenDist <= orignDist);
}

TEST_F(ParabolicSmootherTests, doBlend)
{
   Interpolated nonStraightLine(mStateSpace, mInterpolator);

   Eigen::Vector2d maxVelocity(10., 10.), maxAcceleration(20., 20.);

   auto state = mStateSpace->createState();
   Vector2d p1(1., 1.), p2(2., 1.), p3(2., 2.),
           p4(3., 2.), p5(3., 3.), p6(4.,3.);
   state.setValue(p1);
   nonStraightLine.addWaypoint(0., state);
   state.setValue(p2);
   nonStraightLine.addWaypoint(0.25, state);
   state.setValue(p3);
   nonStraightLine.addWaypoint(0.5, state);
   state.setValue(p4);
   nonStraightLine.addWaypoint(0.75, state);
   state.setValue(p5);
   nonStraightLine.addWaypoint(1., state);
   state.setValue(p6);
   nonStraightLine.addWaypoint(1.25, state);

   std::shared_ptr<Satisfied> testable =
           std::make_shared<Satisfied>(mStateSpace);

   double timelimit = 10.0;
   auto smoothedTrajectory = doBlend(
       nonStraightLine,
       testable,
       maxVelocity,
       maxAcceleration,
       timelimit);

   // Position.
   smoothedTrajectory->evaluate(smoothedTrajectory->getStartTime(), state);
   EXPECT_TRUE(p1.isApprox(state.getValue()));

   smoothedTrajectory->evaluate(smoothedTrajectory->getEndTime(), state);
   Eigen::VectorXd vec;
   mStateSpace->logMap(state, vec);
   EXPECT_TRUE(p6.isApprox(state.getValue()));

   //EXPECT_TRUE(shortenDist <= orignDist);

   double originTime = nonStraightLine.getDuration();
   double shortenTime = smoothedTrajectory->getDuration();
   EXPECT_TRUE(shortenTime<=originTime);
}

TEST_F(ParabolicSmootherTests, doShortcutAndBlend)
{
   Interpolated nonStraightLine(mStateSpace, mInterpolator);

   Eigen::Vector2d maxVelocity(10., 10.), maxAcceleration(20., 20.);

   auto state = mStateSpace->createState();
   Vector2d p1(1., 1.), p2(1., 2.), p3(2., 2.),
           p4(2., 3.), p5(3., 3.);
   state.setValue(p1);
   nonStraightLine.addWaypoint(0., state);
   state.setValue(p2);
   nonStraightLine.addWaypoint(0.25, state);
   state.setValue(p3);
   nonStraightLine.addWaypoint(0.5, state);
   state.setValue(p4);
   nonStraightLine.addWaypoint(0.75, state);
   state.setValue(p5);
   nonStraightLine.addWaypoint(1., state);

   double orignDist = (p2-p1).norm() + (p3-p2).norm()
           + (p4-p3).norm() + (p5-p4).norm();

   std::shared_ptr<Satisfied> testable =
           std::make_shared<Satisfied>(mStateSpace);

   double timelimit = 10.0;
   auto smoothedTrajectory = doShortcutAndBlend(
       nonStraightLine,
       testable,
       maxVelocity,
       maxAcceleration,
       timelimit);

   double shortenDist = 0.0;
   auto currState = mStateSpace->createState();
   auto nextState = mStateSpace->createState();
   Eigen::VectorXd currVec, nextVec;
   for(size_t i=0; i<smoothedTrajectory->getNumWaypoints()-1;++i)
   {
       smoothedTrajectory->getWaypoint(i, currState);
       smoothedTrajectory->getWaypoint(i+1, nextState);
       mStateSpace->logMap(currState, currVec);
       mStateSpace->logMap(nextState, nextVec);
       shortenDist += (nextVec-currVec).norm();
   }

   // Position.
   smoothedTrajectory->evaluate(smoothedTrajectory->getStartTime(), state);
   EXPECT_TRUE(p1.isApprox(state.getValue()));

   smoothedTrajectory->evaluate(smoothedTrajectory->getEndTime(), state);
   EXPECT_TRUE(p5.isApprox(state.getValue()));

   EXPECT_TRUE(shortenDist <= orignDist);

   double originTime = nonStraightLine.getDuration();
   double shortenTime = smoothedTrajectory->getDuration();
   EXPECT_TRUE(shortenTime<=originTime);
}
