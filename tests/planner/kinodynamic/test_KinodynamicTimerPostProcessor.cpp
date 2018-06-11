#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/planner/kinodynamic/KinodynamicTimer.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include "eigen_tests.hpp"

using Eigen::Vector2d;
using aikido::trajectory::Interpolated;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R2;
using aikido::planner::parabolic::convertToSpline;
using aikido::planner::kinodynamic::KinodynamicTimer;

class KinodynamicTimerPostProcessorTests : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mRng = aikido::common::RNGWrapper<std::mt19937>(0);
    mStateSpace = std::make_shared<R2>();
    mInterpolator = std::make_shared<GeodesicInterpolator>(mStateSpace);

    mMaxDeviation = 1e-1;
    mTimeStep = 0.01;
  }

  aikido::common::RNGWrapper<std::mt19937> mRng;
  std::shared_ptr<R2> mStateSpace;
  std::shared_ptr<GeodesicInterpolator> mInterpolator;

  double mMaxDeviation;
  double mTimeStep;
};

TEST_F(KinodynamicTimerPostProcessorTests, testTime)
{
  KinodynamicTimer testKinodynamicTimerPostProcessor(
      Vector2d::Constant(2.), Vector2d::Constant(1.), mMaxDeviation, mTimeStep);

  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd tangentVector;

  // The optimal timing of this trajectory should be a triangle centered at t =
  // 1that accelerates at 1 rad/s^2 for 1 s, then deaccelerates at -1 rad/s^2
  // for 1 s. This corresponds to moving each axis through 2 rad.
  state.setValue(Vector2d(1., 2.));
  inputTrajectory.addWaypoint(0., state);

  state.setValue(Vector2d(2., 3.));
  inputTrajectory.addWaypoint(2., state);

  auto timedTrajectory
      = testKinodynamicTimerPostProcessor.postprocess(inputTrajectory, mRng);

  // TODO: Why does this return three derivatives instead of two?
  EXPECT_GE(timedTrajectory->getNumDerivatives(), 2);
  double durationTolerance = 1e-6;
  EXPECT_NEAR(2., timedTrajectory->getDuration(), durationTolerance);

  // Position.
  timedTrajectory->evaluate(0., state);
  EXPECT_TRUE(Vector2d(1.0, 2.0).isApprox(state.getValue()));

  timedTrajectory->evaluate(1., state);
  EXPECT_TRUE(Vector2d(1.5, 2.5).isApprox(state.getValue()));

  timedTrajectory->evaluate(2., state);
  EXPECT_TRUE(Vector2d(2.0, 3.0).isApprox(state.getValue()));

  // Velocity
  timedTrajectory->evaluateDerivative(0.5, 1, tangentVector);
  EXPECT_TRUE(Vector2d(0.5, 0.5).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(1.0, 1, tangentVector);
  EXPECT_TRUE(Vector2d(1.0, 1.0).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(1.5, 1, tangentVector);
  EXPECT_TRUE(Vector2d(0.5, 0.5).isApprox(tangentVector));

  double accelerationPrecision = 1e-10;
  // Acceleration.
  timedTrajectory->evaluateDerivative(0.5, 2, tangentVector);
  EXPECT_TRUE(Vector2d(1., 1.).isApprox(tangentVector, accelerationPrecision));

  timedTrajectory->evaluateDerivative(1.5, 2, tangentVector);
  EXPECT_TRUE(
      Vector2d(-1., -1.).isApprox(tangentVector, accelerationPrecision));
}

TEST_F(KinodynamicTimerPostProcessorTests, testSplineTiming)
{
  KinodynamicTimer testKinodynamicTimerPostProcessor(
      Vector2d::Constant(2.), Vector2d::Constant(1.), mMaxDeviation, mTimeStep);

  Interpolated interpolated(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  auto state2 = mStateSpace->createState();

  // This is the same test as StraightLine_TriangularProfile, except that the
  // trajectory starts at a non-zero time.
  state.setValue(Vector2d(1., 2.));
  interpolated.addWaypoint(1., state);

  state.setValue(Vector2d(2., 3.));
  interpolated.addWaypoint(3., state);

  auto spline = convertToSpline(interpolated);

  auto timedInterpolated
      = testKinodynamicTimerPostProcessor.postprocess(interpolated, mRng);
  auto timedSpline
      = testKinodynamicTimerPostProcessor.postprocess(*spline, mRng);

  timedInterpolated->evaluate(1., state);
  timedSpline->evaluate(1., state2);
  EXPECT_TRUE(state2.getValue().isApprox(state.getValue()));

  timedInterpolated->evaluate(2., state);
  timedSpline->evaluate(2., state2);
  EXPECT_TRUE(state2.getValue().isApprox(state.getValue()));

  timedInterpolated->evaluate(3., state);
  timedSpline->evaluate(3., state2);
  EXPECT_TRUE(state2.getValue().isApprox(state.getValue()));
}
