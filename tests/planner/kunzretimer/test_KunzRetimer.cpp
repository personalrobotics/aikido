#include <gtest/gtest.h>

#include <aikido/planner/kunzretimer/KunzRetimer.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/trajectory/util.hpp>

#include "eigen_tests.hpp"

using aikido::planner::kunzretimer::computeKunzTiming;
using aikido::statespace::CartesianProduct;
using aikido::statespace::ConstStateSpacePtr;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R1;
using aikido::statespace::SO2;
using aikido::statespace::StateSpacePtr;
using aikido::tests::CompareEigenMatrices;
using aikido::trajectory::convertToSpline;
using aikido::trajectory::Interpolated;
using Eigen::Vector2d;
using Eigen::Vector3d;

class KunzRetimerTests : public ::testing::Test
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  void SetUp() override
  {
    std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
    for (std::size_t i = 0; i < 2; ++i)
      subspaces.emplace_back(std::make_shared<R1>());
    mStateSpace = std::make_shared<CartesianProduct>(subspaces);

    mMaxVelocity = Eigen::Vector2d(1., 1.);
    mMaxAcceleration = Eigen::Vector2d(2., 2.);

    mInterpolator = std::make_shared<GeodesicInterpolator>(mStateSpace);
    mStraightLine = std::make_shared<Interpolated>(mStateSpace, mInterpolator);

    auto state = mStateSpace->createState();
    Eigen::VectorXd positions(2);

    positions << 1.0, 2.0;
    mStateSpace->expMap(positions, state);
    mStraightLine->addWaypoint(0., state);

    positions << 3.0, 4.0;
    mStateSpace->expMap(positions, state);
    mStraightLine->addWaypoint(1., state);
  }

  std::shared_ptr<CartesianProduct> mStateSpace;
  Eigen::Vector2d mMaxVelocity;
  Eigen::Vector2d mMaxAcceleration;

  std::shared_ptr<GeodesicInterpolator> mInterpolator;
  std::shared_ptr<Interpolated> mStraightLine;
};

TEST_F(KunzRetimerTests, SupportedCartesianProduct_DoesNotThrow)
{
  EXPECT_NO_THROW(
      { computeKunzTiming(*mStraightLine, mMaxVelocity, mMaxAcceleration); });
}

TEST_F(KunzRetimerTests, MaxVelocityIsZero_Throws)
{
  Vector2d zeroMaxVelocity(1., 0.);
  EXPECT_THROW(
      { computeKunzTiming(*mStraightLine, zeroMaxVelocity, mMaxAcceleration); },
      std::invalid_argument);
}

TEST_F(KunzRetimerTests, MaxVelocityIsNegative_Throws)
{
  Vector2d negativeMaxVelocity(1., -1);
  EXPECT_THROW(
      {
        computeKunzTiming(
            *mStraightLine, negativeMaxVelocity, mMaxAcceleration);
      },
      std::invalid_argument);
}

TEST_F(KunzRetimerTests, MaxAccelerationIsZero_Throws)
{
  Vector2d zeroMaxAcceleration(1., 0.);
  EXPECT_THROW(
      { computeKunzTiming(*mStraightLine, mMaxVelocity, zeroMaxAcceleration); },
      std::invalid_argument);
}

TEST_F(KunzRetimerTests, MaxAccelerationIsNegative_Throws)
{
  Vector2d negativeMaxAcceleration(1., -1);
  EXPECT_THROW(
      {
        computeKunzTiming(
            *mStraightLine, mMaxVelocity, negativeMaxAcceleration);
      },
      std::invalid_argument);
}

TEST_F(KunzRetimerTests, StartsAtNonZeroTime)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  // This is the same test as StraightLine_TriangularProfile, except that the
  // trajectory starts at a non-zero time.
  Eigen::VectorXd positions(2);
  positions << 1, 2;
  mStateSpace->expMap(positions, state);
  inputTrajectory.addWaypoint(1., state);

  positions << 2, 3;
  mStateSpace->expMap(positions, state);
  inputTrajectory.addWaypoint(3., state);

  auto timedTrajectory = computeKunzTiming(
      inputTrajectory, Vector2d::Constant(2.), Vector2d::Constant(1.));
  EXPECT_FALSE(timedTrajectory == nullptr) << "Trajectory timing failed";

  timedTrajectory->evaluate(1., state);
  mStateSpace->logMap(state, positions);
  EXPECT_EIGEN_EQUAL(Vector2d(1.0, 2.0), positions, 1e-6);

  timedTrajectory->evaluate(2., state);
  mStateSpace->logMap(state, positions);
  EXPECT_EIGEN_EQUAL(Vector2d(1.5, 2.5), positions, 1e-6);

  timedTrajectory->evaluate(3., state);
  mStateSpace->logMap(state, positions);
  EXPECT_EIGEN_EQUAL(Vector2d(2.0, 3.0), positions, 1e-6);
}

TEST_F(KunzRetimerTests, StraightLine_TriangularProfile)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd positions(2);
  Eigen::VectorXd tangentVector;

  double maxDeviation = 1e-2;
  double timeStep = 0.1;
  // The optimal timing of this trajectory should be a triangle centered at t =
  // 1that accelerates at 1 rad/s^2 for 1 s, then deaccelerates at -1 rad/s^2
  // for 1 s. This corresponds to moving each axis through 2 rad.
  positions << 1, 2;
  mStateSpace->expMap(positions, state);
  inputTrajectory.addWaypoint(0., state);

  positions << 2, 3;
  mStateSpace->expMap(positions, state);
  inputTrajectory.addWaypoint(2., state);

  auto timedTrajectory = computeKunzTiming(
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
  mStateSpace->logMap(state, positions);
  EXPECT_EIGEN_EQUAL(Vector2d(1.0, 2.0), positions, 1e-6);

  timedTrajectory->evaluate(1., state);
  mStateSpace->logMap(state, positions);
  EXPECT_EIGEN_EQUAL(Vector2d(1.5, 2.5), positions, 1e-6);

  timedTrajectory->evaluate(2., state);
  mStateSpace->logMap(state, positions);
  EXPECT_EIGEN_EQUAL(Vector2d(2.0, 3.0), positions, 1e-6);

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

TEST_F(KunzRetimerTests, StraightLine_TrapezoidalProfile)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd positions(2);
  Eigen::VectorXd tangentVector;

  // The optimal timing of this trajectory should be a trapezoid that:
  // - accelerates at 1 rad/s^2 for 1 s
  // - coasts at 1 m/s for 1 s
  // - deaccelerates at -1 rad/s^2 for 1 s
  positions << 1, 2;
  mStateSpace->expMap(positions, state);
  inputTrajectory.addWaypoint(0., state);

  positions << 3, 4;
  mStateSpace->expMap(positions, state);
  inputTrajectory.addWaypoint(2., state);

  double maxDeviation = 1e-2;
  double timeStep = 0.1;
  auto timedTrajectory = computeKunzTiming(
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
  mStateSpace->logMap(state, positions);
  EXPECT_EIGEN_EQUAL(Vector2d(1.0, 2.0), positions, 1e-6);

  timedTrajectory->evaluate(1., state);
  mStateSpace->logMap(state, positions);
  EXPECT_EIGEN_EQUAL(Vector2d(1.5, 2.5), positions, 1e-6);

  timedTrajectory->evaluate(2., state);
  mStateSpace->logMap(state, positions);
  EXPECT_EIGEN_EQUAL(Vector2d(2.5, 3.5), positions, 1e-6);

  timedTrajectory->evaluate(3., state);
  mStateSpace->logMap(state, positions);
  EXPECT_EIGEN_EQUAL(Vector2d(3.0, 4.0), positions, 1e-6);

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

TEST_F(KunzRetimerTests, StraightLine_DifferentAccelerationLimits)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd positions(2);
  Eigen::VectorXd tangentVector;

  // The optimal timing of this trajectory should be a trapezoid that:
  // - accelerates at 1 rad/s^2 for 1 s
  // - coasts at 1 m/s for 1 s
  // - deaccelerates at -1 rad/s^2 for 1 s
  //
  // Note that the second dimension of the state space could result in a faster
  // timing by executing a triangular velocity profile. This is not possible
  // because the first dimension has a lower acceleration limit.
  positions << 1, 2;
  mStateSpace->expMap(positions, state);
  inputTrajectory.addWaypoint(0., state);

  positions << 3, 4;
  mStateSpace->expMap(positions, state);
  inputTrajectory.addWaypoint(2., state);

  auto timedTrajectory
      = computeKunzTiming(inputTrajectory, Vector2d(1., 2.), Vector2d(1., 1.));

  double durationTolerance = 1e-6;
  EXPECT_GE(timedTrajectory->getNumDerivatives(), 2);
  EXPECT_NEAR(3., timedTrajectory->getDuration(), durationTolerance);
}

TEST_F(KunzRetimerTests, timingArbitraryMultipleWaypoints)
{
  std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
  for (std::size_t i = 0; i < 4; ++i)
    subspaces.emplace_back(std::make_shared<R1>());
  auto stateSpace = std::make_shared<CartesianProduct>(subspaces);

  auto interpolator = std::make_shared<GeodesicInterpolator>(stateSpace);
  Interpolated inputTrajectory(stateSpace, interpolator);

  Eigen::VectorXd waypoint(4);
  auto state = stateSpace->createState();

  waypoint << 1427.0, 368.0, 690.0, 90.0;
  stateSpace->expMap(waypoint, state);
  inputTrajectory.addWaypoint(0., state);

  waypoint << 1427.0, 368.0, 790.0, 90.0;
  stateSpace->expMap(waypoint, state);
  inputTrajectory.addWaypoint(1., state);

  waypoint << 952.0, 433.0, 1051.0, 90.0;
  stateSpace->expMap(waypoint, state);
  inputTrajectory.addWaypoint(2., state);

  waypoint << 452.5, 533.0, 1051.0, 90.0;
  stateSpace->expMap(waypoint, state);
  inputTrajectory.addWaypoint(3., state);

  waypoint << 452.5, 533.0, 951.0, 90.0;
  stateSpace->expMap(waypoint, state);
  inputTrajectory.addWaypoint(4., state);

  Eigen::VectorXd maxVelocities(4);
  maxVelocities << 10., 10, 10, 10;
  Eigen::VectorXd maxAccelerations(4);
  maxAccelerations << 10., 10., 10., 10.;

  double maxDeviation = 1.;
  double timeStep = .2;
  EXPECT_NO_THROW({
    computeKunzTiming(
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
