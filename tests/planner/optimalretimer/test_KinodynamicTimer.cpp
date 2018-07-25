#include <gtest/gtest.h>
#include <aikido/planner/optimalretimer/KinodynamicTimer.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include "eigen_tests.hpp"

using Eigen::Vector2d;
using Eigen::Vector3d;
using aikido::trajectory::Interpolated;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R2;
using aikido::statespace::CartesianProduct;
using aikido::statespace::SO2;
using aikido::statespace::StateSpacePtr;
using aikido::statespace::ConstStateSpacePtr;
using aikido::planner::optimalretimer::computeKinodynamicTiming;
using aikido::planner::parabolic::convertToSpline;
using aikido::tests::CompareEigenMatrices;

class KinodynamicTimerTests : public ::testing::Test
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
    mStraightLine = std::make_shared<Interpolated>(mStateSpace, mInterpolator);

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

TEST_F(KinodynamicTimerTests, MaxVelocityIsZero_Throws)
{
  Vector2d zeroMaxVelocity(1., 0.);
  EXPECT_THROW(
      {
        computeKinodynamicTiming(
            *mStraightLine, zeroMaxVelocity, mMaxAcceleration);
      },
      std::invalid_argument);
}

TEST_F(KinodynamicTimerTests, MaxVelocityIsNegative_Throws)
{
  Vector2d negativeMaxVelocity(1., -1);
  EXPECT_THROW(
      {
        computeKinodynamicTiming(
            *mStraightLine, negativeMaxVelocity, mMaxAcceleration);
      },
      std::invalid_argument);
}

TEST_F(KinodynamicTimerTests, MaxAccelerationIsZero_Throws)
{
  Vector2d zeroMaxAcceleration(1., 0.);
  EXPECT_THROW(
      {
        computeKinodynamicTiming(
            *mStraightLine, mMaxVelocity, zeroMaxAcceleration);
      },
      std::invalid_argument);
}

TEST_F(KinodynamicTimerTests, MaxAccelerationIsNegative_Throws)
{
  Vector2d negativeMaxAcceleration(1., -1);
  EXPECT_THROW(
      {
        computeKinodynamicTiming(
            *mStraightLine, mMaxVelocity, negativeMaxAcceleration);
      },
      std::invalid_argument);
}

TEST_F(KinodynamicTimerTests, StartsAtNonZeroTime)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();

  // This is the same test as StraightLine_TriangularProfile, except that the
  // trajectory starts at a non-zero time.
  state.setValue(Vector2d(1., 2.));
  inputTrajectory.addWaypoint(1., state);

  state.setValue(Vector2d(2., 3.));
  inputTrajectory.addWaypoint(3., state);

  auto timedTrajectory = computeKinodynamicTiming(
      inputTrajectory, Vector2d::Constant(2.), Vector2d::Constant(1.));

  EXPECT_FALSE(timedTrajectory == nullptr) << "Trajectory timing failed";

  timedTrajectory->evaluate(1., state);
  EXPECT_EIGEN_EQUAL(Vector2d(1.0, 2.0), state.getValue(), 1e-6);

  timedTrajectory->evaluate(2., state);
  EXPECT_EIGEN_EQUAL(Vector2d(1.5, 2.5), state.getValue(), 1e-6);

  timedTrajectory->evaluate(3., state);
  EXPECT_EIGEN_EQUAL(Vector2d(2.0, 3.0), state.getValue(), 1e-6);
}

TEST_F(KinodynamicTimerTests, InterploatedSplineEquivalence)
{
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

  auto timedInterpolated = computeKinodynamicTiming(
      interpolated, Vector2d::Constant(2.), Vector2d::Constant(1.));
  auto timedSpline = computeKinodynamicTiming(
      *spline, Vector2d::Constant(2.), Vector2d::Constant(1.));

  timedInterpolated->evaluate(1., state);
  timedSpline->evaluate(1., state2);
  EXPECT_EIGEN_EQUAL(state2.getValue(), state.getValue(), 1e-6);

  timedInterpolated->evaluate(2., state);
  timedSpline->evaluate(2., state2);
  EXPECT_EIGEN_EQUAL(state2.getValue(), state.getValue(), 1e-6);

  timedInterpolated->evaluate(3., state);
  timedSpline->evaluate(3., state2);
  EXPECT_EIGEN_EQUAL(state2.getValue(), state.getValue(), 1e-6);
}

TEST_F(KinodynamicTimerTests, StraightLine_TriangularProfile)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd tangentVector;

  double maxDeviation = 1e-2;
  double timeStep = 0.1;
  // The optimal timing of this trajectory should be a triangle centered at t =
  // 1that accelerates at 1 rad/s^2 for 1 s, then deaccelerates at -1 rad/s^2
  // for 1 s. This corresponds to moving each axis through 2 rad.
  state.setValue(Vector2d(1., 2.));
  inputTrajectory.addWaypoint(0., state);

  state.setValue(Vector2d(2., 3.));
  inputTrajectory.addWaypoint(2., state);

  auto timedTrajectory = computeKinodynamicTiming(
      inputTrajectory,
      Vector2d::Constant(2.),
      Vector2d::Constant(1.),
      maxDeviation,
      timeStep);

  double durationTolerance = 1e-6;
  // TODO: Why does this return three derivatives instead of two?
  EXPECT_GE(timedTrajectory->getNumDerivatives(), 2);
  EXPECT_NEAR(2., timedTrajectory->getDuration(), durationTolerance);

  // Position.
  timedTrajectory->evaluate(0., state);
  EXPECT_EIGEN_EQUAL(Vector2d(1.0, 2.0), state.getValue(), 1e-6);

  timedTrajectory->evaluate(1., state);
  EXPECT_EIGEN_EQUAL(Vector2d(1.5, 2.5), state.getValue(), 1e-6);

  timedTrajectory->evaluate(2., state);
  EXPECT_EIGEN_EQUAL(Vector2d(2.0, 3.0), state.getValue(), 1e-6);

  // Velocity
  timedTrajectory->evaluateDerivative(0.5, 1, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(0.5, 0.5), tangentVector, 1e-6);

  timedTrajectory->evaluateDerivative(1.0, 1, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(1.0, 1.0), tangentVector, 1e-6);

  timedTrajectory->evaluateDerivative(1.5, 1, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(0.5, 0.5), tangentVector, 1e-6);

  // Acceleration.
  timedTrajectory->evaluateDerivative(0.5, 2, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(1., 1.), tangentVector, 1e-6);

  timedTrajectory->evaluateDerivative(1.5, 2, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(-1., -1.), tangentVector, 1e-6);
}

TEST_F(KinodynamicTimerTests, StraightLine_TrapezoidalProfile)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd tangentVector;

  // The optimal timing of this trajectory should be a trapezoid that:
  // - accelerates at 1 rad/s^2 for 1 s
  // - coasts at 1 m/s for 1 s
  // - deaccelerates at -1 rad/s^2 for 1 s
  state.setValue(Vector2d(1., 2.));
  inputTrajectory.addWaypoint(0., state);

  state.setValue(Vector2d(3., 4.));
  inputTrajectory.addWaypoint(2., state);

  double maxDeviation = 1e-2;
  double timeStep = 0.1;
  auto timedTrajectory = computeKinodynamicTiming(
      inputTrajectory,
      Vector2d::Constant(1.),
      Vector2d::Constant(1.),
      maxDeviation,
      timeStep);

  // TODO: Why does this return three derivatives instead of two?
  double durationTolerance = 1e-6;
  EXPECT_GE(timedTrajectory->getNumDerivatives(), 2);
  EXPECT_NEAR(3., timedTrajectory->getDuration(), durationTolerance);

  // Position.
  timedTrajectory->evaluate(0., state);
  EXPECT_EIGEN_EQUAL(Vector2d(1.0, 2.0), state.getValue(), 1e-6);

  timedTrajectory->evaluate(1., state);
  EXPECT_EIGEN_EQUAL(Vector2d(1.5, 2.5), state.getValue(), 1e-6);

  timedTrajectory->evaluate(2., state);
  EXPECT_EIGEN_EQUAL(Vector2d(2.5, 3.5), state.getValue(), 1e-6);

  timedTrajectory->evaluate(3., state);
  EXPECT_EIGEN_EQUAL(Vector2d(3.0, 4.0), state.getValue(), 1e-6);

  // Velocity
  timedTrajectory->evaluateDerivative(0.5, 1, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(0.5, 0.5), tangentVector, 1e-6);

  timedTrajectory->evaluateDerivative(1.0, 1, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(1.0, 1.0), tangentVector, 1e-6);

  timedTrajectory->evaluateDerivative(1.5, 1, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(1.0, 1.0), tangentVector, 1e-6);

  timedTrajectory->evaluateDerivative(2.0, 1, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(1.0, 1.0), tangentVector, 1e-6);

  timedTrajectory->evaluateDerivative(2.5, 1, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(0.5, 0.5), tangentVector, 1e-6);

  timedTrajectory->evaluateDerivative(3.0, 1, tangentVector);
  // TODO: isApprox does not work when comparing against zero.
  // EXPECT_TRUE(Vector2d(0.0, 0.0).isApprox(tangentVector));

  // Acceleration.
  timedTrajectory->evaluateDerivative(0.5, 2, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(1., 1.), tangentVector, 1e-6);

  timedTrajectory->evaluateDerivative(1.5, 2, tangentVector);
  // TODO: isApprox does not work when comparing against zero.
  // EXPECT_TRUE(Vector2d(0., 0.).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(2.5, 2, tangentVector);
  EXPECT_EIGEN_EQUAL(Vector2d(-1., -1.), tangentVector, 1e-6);
}

TEST_F(KinodynamicTimerTests, StraightLine_DifferentAccelerationLimits)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd tangentVector;

  // The optimal timing of this trajectory should be a trapezoid that:
  // - accelerates at 1 rad/s^2 for 1 s
  // - coasts at 1 m/s for 1 s
  // - deaccelerates at -1 rad/s^2 for 1 s
  //
  // Note that the second dimension of the state space could result in a faster
  // timing by executing a triangular velocity profile. This is not possible
  // because the first dimension has a lower acceleration limit.
  state.setValue(Vector2d(1., 2.));
  inputTrajectory.addWaypoint(0., state);

  state.setValue(Vector2d(3., 4.));
  inputTrajectory.addWaypoint(2., state);

  auto timedTrajectory = computeKinodynamicTiming(
      inputTrajectory, Vector2d(1., 2.), Vector2d(1., 1.));

  double durationTolerance = 1e-6;
  EXPECT_GE(timedTrajectory->getNumDerivatives(), 2);
  EXPECT_NEAR(3., timedTrajectory->getDuration(), durationTolerance);
}

TEST_F(KinodynamicTimerTests, SupportedCartesianProduct_DoesNotThrow)
{
  auto stateSpace = std::make_shared<CartesianProduct>(
      std::vector<ConstStateSpacePtr>{
          std::make_shared<R2>(), std::make_shared<SO2>(),
      });
  auto state = stateSpace->createState();

  std::shared_ptr<GeodesicInterpolator> interpolator
      = std::make_shared<GeodesicInterpolator>(stateSpace);

  Interpolated inputTrajectory(stateSpace, interpolator);

  state.getSubStateHandle<R2>(0).setValue(Vector2d::Zero());
  state.getSubStateHandle<SO2>(1).fromAngle(0.);
  inputTrajectory.addWaypoint(0., state);

  state.getSubStateHandle<R2>(0).setValue(Vector2d::Zero());
  state.getSubStateHandle<SO2>(1).fromAngle(M_PI_2);
  inputTrajectory.addWaypoint(1., state);

  EXPECT_NO_THROW({
    computeKinodynamicTiming(
        inputTrajectory, Vector3d::Ones(), Vector3d::Ones());
  });
}

TEST_F(KinodynamicTimerTests, timingAribtraryMultipleWaypoints)
{
  auto stateSpace = std::make_shared<aikido::statespace::R<4>>();
  auto interpolator = std::make_shared<GeodesicInterpolator>(stateSpace);
  Interpolated inputTrajectory(stateSpace, interpolator);

  Eigen::VectorXd waypoint(4);
  auto state = stateSpace->createState();

  waypoint << 1427.0, 368.0, 690.0, 90.0;
  state.setValue(waypoint);
  inputTrajectory.addWaypoint(0., state);
  waypoint << 1427.0, 368.0, 790.0, 90.0;
  state.setValue(waypoint);
  inputTrajectory.addWaypoint(1., state);
  waypoint << 952.499938964844, 433.0, 1051.0, 90.0;
  state.setValue(waypoint);
  inputTrajectory.addWaypoint(2., state);
  waypoint << 452.5, 533.0, 1051.0, 90.0;
  state.setValue(waypoint);
  inputTrajectory.addWaypoint(3., state);
  waypoint << 452.5, 533.0, 951.0, 90.0;
  state.setValue(waypoint);
  inputTrajectory.addWaypoint(4., state);

  Eigen::VectorXd maxVelocities(4);
  maxVelocities << 1.3, 0.67, 0.67, 0.5;
  Eigen::VectorXd maxAccelerations(4);
  maxAccelerations << 0.002, 0.002, 0.002, 0.002;

  double maxDeviation = 10.;
  double timeStep = 10.;
  EXPECT_NO_THROW({
    computeKinodynamicTiming(
        inputTrajectory,
        maxVelocities,
        maxAccelerations,
        maxDeviation,
        timeStep);
  });
}

// TODO: Test what happens when two waypoints are coincident.
// TODO: Add a test for different velocity limits.
// TODO: Add a test where DOFs have different ramp transition points.
