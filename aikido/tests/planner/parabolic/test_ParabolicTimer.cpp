#include <gtest/gtest.h>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/SO3.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>

using Eigen::Vector2d;
using Eigen::Vector3d;
using aikido::trajectory::Interpolated;
using aikido::planner::parabolic::computeParabolicTiming;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::Rn;
using aikido::statespace::CartesianProduct;
using aikido::statespace::SO2;
using aikido::statespace::SO3;
using aikido::statespace::StateSpacePtr;

class ParabolicTimerTests : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mStateSpace = std::make_shared<Rn>(2);
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

  std::shared_ptr<Rn> mStateSpace;
  Eigen::Vector2d mMaxVelocity;
  Eigen::Vector2d mMaxAcceleration;

  std::shared_ptr<GeodesicInterpolator> mInterpolator;
  std::shared_ptr<Interpolated> mStraightLine;
};

TEST_F(ParabolicTimerTests, InputTrajectoryIsEmpty_Throws)
{
  Interpolated emptyTrajectory(mStateSpace, mInterpolator);

  EXPECT_THROW({
    computeParabolicTiming(emptyTrajectory, mMaxVelocity, mMaxAcceleration);
  }, std::invalid_argument);
}

TEST_F(ParabolicTimerTests, MaxVelocityIsZero_Throws)
{
  Vector2d zeroMaxVelocity(1., 0.);

  EXPECT_THROW({
    computeParabolicTiming(
      *mStraightLine, zeroMaxVelocity, mMaxAcceleration);
  }, std::invalid_argument);
}

TEST_F(ParabolicTimerTests, MaxVelocityIsNegative_Throws)
{
  Vector2d negativeMaxVelocity(1., -1);

  EXPECT_THROW({
    computeParabolicTiming(
      *mStraightLine, negativeMaxVelocity, mMaxAcceleration);
  }, std::invalid_argument);
}

TEST_F(ParabolicTimerTests, MaxAccelerationIsZero_Throws)
{
  Vector2d zeroMaxAcceleration(1., 0.);

  EXPECT_THROW({
    computeParabolicTiming(
      *mStraightLine, mMaxVelocity, zeroMaxAcceleration);
  }, std::invalid_argument);
}

TEST_F(ParabolicTimerTests, MaxAccelerationIsNegative_Throws)
{
  Vector2d negativeMaxAcceleration(1., -1);

  EXPECT_THROW({
    computeParabolicTiming(
      *mStraightLine, mMaxVelocity, negativeMaxAcceleration);
  }, std::invalid_argument);
}

TEST_F(ParabolicTimerTests, StartsAtNonZeroTime)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd tangentVector;

  // This is the same test as StraightLine_TriangularProfile, except that the
  // trajectory starts at a non-zero time.
  state.setValue(Vector2d(1., 2.));
  inputTrajectory.addWaypoint(1., state);

  state.setValue(Vector2d(2., 3.));
  inputTrajectory.addWaypoint(3., state);

  auto timedTrajectory = computeParabolicTiming(
    inputTrajectory, Vector2d::Constant(2.), Vector2d::Constant(1.));

  timedTrajectory->evaluate(1., state);
  EXPECT_TRUE(Vector2d(1.0, 2.0).isApprox(state.getValue()));

  timedTrajectory->evaluate(2., state);
  EXPECT_TRUE(Vector2d(1.5, 2.5).isApprox(state.getValue()));

  timedTrajectory->evaluate(3., state);
  EXPECT_TRUE(Vector2d(2.0, 3.0).isApprox(state.getValue()));

}

TEST_F(ParabolicTimerTests, StraightLine_TriangularProfile)
{
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

  auto timedTrajectory = computeParabolicTiming(
    inputTrajectory, Vector2d::Constant(2.), Vector2d::Constant(1.));

  // TODO: Why does this return three derivatives instead of two?
  EXPECT_GE(timedTrajectory->getNumDerivatives(), 2);
  EXPECT_EQ(2, timedTrajectory->getNumSegments());
  EXPECT_DOUBLE_EQ(2., timedTrajectory->getDuration());

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

  // Acceleration.
  timedTrajectory->evaluateDerivative(0.5, 2, tangentVector);
  EXPECT_TRUE(Vector2d(1., 1.).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(1.5, 2, tangentVector);
  EXPECT_TRUE(Vector2d(-1., -1.).isApprox(tangentVector));
}

TEST_F(ParabolicTimerTests, StraightLine_TrapezoidalProfile)
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

  auto timedTrajectory = computeParabolicTiming(
    inputTrajectory, Vector2d::Constant(1.), Vector2d::Constant(1.));

  // TODO: Why does this return three derivatives instead of two?
  EXPECT_GE(timedTrajectory->getNumDerivatives(), 2);
  EXPECT_EQ(3, timedTrajectory->getNumSegments());
  EXPECT_DOUBLE_EQ(3., timedTrajectory->getDuration());

  // Position.
  timedTrajectory->evaluate(0., state);
  EXPECT_TRUE(Vector2d(1.0, 2.0).isApprox(state.getValue()));

  timedTrajectory->evaluate(1., state);
  EXPECT_TRUE(Vector2d(1.5, 2.5).isApprox(state.getValue()));

  timedTrajectory->evaluate(2., state);
  EXPECT_TRUE(Vector2d(2.5, 3.5).isApprox(state.getValue()));

  timedTrajectory->evaluate(3., state);
  EXPECT_TRUE(Vector2d(3.0, 4.0).isApprox(state.getValue()));

  // Velocity
  timedTrajectory->evaluateDerivative(0.5, 1, tangentVector);
  EXPECT_TRUE(Vector2d(0.5, 0.5).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(1.0, 1, tangentVector);
  EXPECT_TRUE(Vector2d(1.0, 1.0).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(1.5, 1, tangentVector);
  EXPECT_TRUE(Vector2d(1.0, 1.0).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(2.0, 1, tangentVector);
  EXPECT_TRUE(Vector2d(1.0, 1.0).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(2.5, 1, tangentVector);
  EXPECT_TRUE(Vector2d(0.5, 0.5).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(3.0, 1, tangentVector);
  // TODO: isApprox does not work when comparing against zero.
  //EXPECT_TRUE(Vector2d(0.0, 0.0).isApprox(tangentVector));

  // Acceleration.
  timedTrajectory->evaluateDerivative(0.5, 2, tangentVector);
  EXPECT_TRUE(Vector2d(1., 1.).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(1.5, 2, tangentVector);
  // TODO: isApprox does not work when comparing against zero.
  //EXPECT_TRUE(Vector2d(0., 0.).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(2.5, 2, tangentVector);
  EXPECT_TRUE(Vector2d(-1., -1.).isApprox(tangentVector));
}

TEST_F(ParabolicTimerTests, StraightLine_DifferentAccelerationLimits)
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

  auto timedTrajectory = computeParabolicTiming(
    inputTrajectory, Vector2d(1., 2.), Vector2d(1., 1.));

  EXPECT_GE(timedTrajectory->getNumDerivatives(), 2);
  EXPECT_EQ(3, timedTrajectory->getNumSegments());
  EXPECT_DOUBLE_EQ(3., timedTrajectory->getDuration());
}

TEST_F(ParabolicTimerTests, UnsupportedStateSpace_Throws)
{
  auto stateSpace = std::make_shared<SO3>();
  auto state = stateSpace->createState();

  Interpolated inputTrajectory(stateSpace, mInterpolator);

  state.setQuaternion(Eigen::Quaterniond::Identity());
  inputTrajectory.addWaypoint(0., state);

  state.setQuaternion(Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Vector3d::UnitX())));
  inputTrajectory.addWaypoint(1., state);

  EXPECT_THROW({
    computeParabolicTiming(
      inputTrajectory, Vector3d::Ones(), Vector3d::Ones());
  }, std::invalid_argument);
}

TEST_F(ParabolicTimerTests, UnsupportedCartesianProduct_Throws)
{
  auto stateSpace = std::make_shared<CartesianProduct>(
    std::vector<StateSpacePtr> { std::make_shared<SO3>() });
  auto state = stateSpace->createState();

  Interpolated inputTrajectory(stateSpace, mInterpolator);

  state.getSubStateHandle<SO3>(0).setQuaternion(
    Eigen::Quaterniond::Identity());
  inputTrajectory.addWaypoint(0., state);

  state.getSubStateHandle<SO3>(0).setQuaternion(Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Vector3d::UnitX())));
  inputTrajectory.addWaypoint(1., state);

  EXPECT_THROW({
    computeParabolicTiming(
      inputTrajectory, Vector3d::Ones(), Vector3d::Ones());
  }, std::invalid_argument);
}

TEST_F(ParabolicTimerTests, SupportedCartesianProduct_DoesNotThrow)
{
  auto stateSpace = std::make_shared<CartesianProduct>(
    std::vector<StateSpacePtr> {
      std::make_shared<Rn>(2),
      std::make_shared<SO2>(),
    });
  auto state = stateSpace->createState();

  Interpolated inputTrajectory(stateSpace, mInterpolator);

  state.getSubStateHandle<Rn>(0).setValue(Vector2d::Zero());
  state.getSubStateHandle<SO2>(1).setAngle(0.);
  inputTrajectory.addWaypoint(0., state);

  state.getSubStateHandle<Rn>(0).setValue(Vector2d::Zero());
  state.getSubStateHandle<SO2>(1).setAngle(M_PI_2);
  inputTrajectory.addWaypoint(1., state);

  EXPECT_NO_THROW({
    computeParabolicTiming(
      inputTrajectory, Vector3d::Ones(), Vector3d::Ones());
  });

}

// TODO: Test what happens when two waypoints are coincident.
// TODO: Add a test for different velocity limits.
// TODO: Add a test where DOFs have different ramp transition points.
