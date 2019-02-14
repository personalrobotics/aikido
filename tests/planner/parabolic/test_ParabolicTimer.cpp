#include <gtest/gtest.h>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/SO3.hpp>
#include <aikido/trajectory/util.hpp>

using Eigen::Vector2d;
using Eigen::Vector3d;
using aikido::trajectory::Interpolated;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R1;
using aikido::statespace::CartesianProduct;
using aikido::statespace::SO2;
using aikido::statespace::SO3;
using aikido::statespace::StateSpacePtr;
using aikido::statespace::ConstStateSpacePtr;
using aikido::trajectory::convertToSpline;
using aikido::planner::parabolic::computeParabolicTiming;

class ParabolicTimerTests : public ::testing::Test
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

    Vector2d p1(1, 2), p2(3, 4);
    mStateSpace->expMap(p1, state);
    mStraightLine->addWaypoint(0., state);

    mStateSpace->expMap(p2, state);
    mStraightLine->addWaypoint(1., state);
  }

  std::shared_ptr<CartesianProduct> mStateSpace;
  Eigen::Vector2d mMaxVelocity;
  Eigen::Vector2d mMaxAcceleration;

  std::shared_ptr<GeodesicInterpolator> mInterpolator;
  std::shared_ptr<Interpolated> mStraightLine;
};

TEST_F(ParabolicTimerTests, SupportedCartesianProduct_DoesNotThrow)
{
  EXPECT_NO_THROW({
    computeParabolicTiming(*mStraightLine, mMaxVelocity, mMaxAcceleration);
  });
}

TEST_F(ParabolicTimerTests, InputTrajectoryIsEmpty_Throws)
{
  Interpolated emptyTrajectory(mStateSpace, mInterpolator);

  EXPECT_THROW({ convertToSpline(emptyTrajectory); }, std::invalid_argument);
}

TEST_F(ParabolicTimerTests, MaxVelocityIsZero_Throws)
{
  Vector2d zeroMaxVelocity(1., 0.);
  EXPECT_THROW(
      {
        computeParabolicTiming(
            *mStraightLine, zeroMaxVelocity, mMaxAcceleration);
      },
      std::invalid_argument);
}

TEST_F(ParabolicTimerTests, MaxVelocityIsNegative_Throws)
{
  Vector2d negativeMaxVelocity(1., -1);
  EXPECT_THROW(
      {
        computeParabolicTiming(
            *mStraightLine, negativeMaxVelocity, mMaxAcceleration);
      },
      std::invalid_argument);
}

TEST_F(ParabolicTimerTests, MaxAccelerationIsZero_Throws)
{
  Vector2d zeroMaxAcceleration(1., 0.);
  EXPECT_THROW(
      {
        computeParabolicTiming(
            *mStraightLine, mMaxVelocity, zeroMaxAcceleration);
      },
      std::invalid_argument);
}

TEST_F(ParabolicTimerTests, MaxAccelerationIsNegative_Throws)
{
  Vector2d negativeMaxAcceleration(1., -1);
  EXPECT_THROW(
      {
        computeParabolicTiming(
            *mStraightLine, mMaxVelocity, negativeMaxAcceleration);
      },
      std::invalid_argument);
}

TEST_F(ParabolicTimerTests, StartsAtNonZeroTime)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd positions;
  Vector2d p1(1, 2), p2(2, 3);

  // This is the same test as StraightLine_TriangularProfile, except that the
  // trajectory starts at a non-zero time.
  mStateSpace->expMap(p1, state);
  inputTrajectory.addWaypoint(1., state);

  mStateSpace->expMap(p2, state);
  inputTrajectory.addWaypoint(3., state);

  auto timedTrajectory = computeParabolicTiming(
      inputTrajectory, Vector2d::Constant(2.), Vector2d::Constant(1.));

  timedTrajectory->evaluate(1., state);
  mStateSpace->logMap(state, positions);
  EXPECT_TRUE(Vector2d(1.0, 2.0).isApprox(positions));

  timedTrajectory->evaluate(2., state);
  mStateSpace->logMap(state, positions);
  EXPECT_TRUE(Vector2d(1.5, 2.5).isApprox(positions));

  timedTrajectory->evaluate(3., state);
  mStateSpace->logMap(state, positions);
  EXPECT_TRUE(Vector2d(2.0, 3.0).isApprox(positions));
}

TEST_F(ParabolicTimerTests, InterploatedSplineEquivalence)
{
  Interpolated interpolated(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  auto state2 = mStateSpace->createState();
  Eigen::VectorXd positions, positions2;

  // This is the same test as StraightLine_TriangularProfile, except that the
  // trajectory starts at a non-zero time.
  Vector2d p1(1, 2), p2(2, 3);

  mStateSpace->expMap(p1, state);
  interpolated.addWaypoint(1., state);

  mStateSpace->expMap(p2, state);
  interpolated.addWaypoint(3., state);

  auto spline = convertToSpline(interpolated);

  auto timedInterpolated = computeParabolicTiming(
      interpolated, Vector2d::Constant(2.), Vector2d::Constant(1.));
  auto timedSpline = computeParabolicTiming(
      *spline, Vector2d::Constant(2.), Vector2d::Constant(1.));

  timedInterpolated->evaluate(1., state);
  timedSpline->evaluate(1., state2);
  mStateSpace->logMap(state, positions);
  mStateSpace->logMap(state2, positions2);
  EXPECT_TRUE(positions2.isApprox(positions));

  timedInterpolated->evaluate(2., state);
  timedSpline->evaluate(2., state2);
  mStateSpace->logMap(state, positions);
  mStateSpace->logMap(state2, positions2);
  EXPECT_TRUE(positions2.isApprox(positions));

  timedInterpolated->evaluate(3., state);
  timedSpline->evaluate(3., state2);
  mStateSpace->logMap(state, positions);
  mStateSpace->logMap(state2, positions2);
  EXPECT_TRUE(positions2.isApprox(positions));
}

TEST_F(ParabolicTimerTests, StraightLine_TriangularProfile)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd tangentVector;
  Vector2d p1(1, 2), p2(2, 3);

  // The optimal timing of this trajectory should be a triangle centered at t =
  // 1that accelerates at 1 rad/s^2 for 1 s, then deaccelerates at -1 rad/s^2
  // for 1 s. This corresponds to moving each axis through 2 rad.
  mStateSpace->expMap(p1, state);
  inputTrajectory.addWaypoint(0., state);

  mStateSpace->expMap(p2, state);
  inputTrajectory.addWaypoint(2., state);

  auto timedTrajectory = computeParabolicTiming(
      inputTrajectory, Vector2d::Constant(2.), Vector2d::Constant(1.));

  // TODO: Why does this return three derivatives instead of two?
  EXPECT_GE(timedTrajectory->getNumDerivatives(), 2);
  EXPECT_EQ(2, timedTrajectory->getNumSegments());
  EXPECT_DOUBLE_EQ(2., timedTrajectory->getDuration());

  Eigen::VectorXd positions;

  // Position.
  timedTrajectory->evaluate(0., state);
  mStateSpace->logMap(state, positions);
  EXPECT_TRUE(Vector2d(1.0, 2.0).isApprox(positions));

  timedTrajectory->evaluate(1., state);
  mStateSpace->logMap(state, positions);
  EXPECT_TRUE(Vector2d(1.5, 2.5).isApprox(positions));

  timedTrajectory->evaluate(2., state);
  mStateSpace->logMap(state, positions);
  EXPECT_TRUE(Vector2d(2.0, 3.0).isApprox(positions));

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
  Vector2d p1(1, 2), p2(3, 4);

  // The optimal timing of this trajectory should be a trapezoid that:
  // - accelerates at 1 rad/s^2 for 1 s
  // - coasts at 1 m/s for 1 s
  // - deaccelerates at -1 rad/s^2 for 1 s
  mStateSpace->expMap(p1, state);
  inputTrajectory.addWaypoint(0., state);

  mStateSpace->expMap(p2, state);
  inputTrajectory.addWaypoint(2., state);

  auto timedTrajectory = computeParabolicTiming(
      inputTrajectory, Vector2d::Constant(1.), Vector2d::Constant(1.));

  // TODO: Why does this return three derivatives instead of two?
  EXPECT_GE(timedTrajectory->getNumDerivatives(), 2);
  EXPECT_EQ(3, timedTrajectory->getNumSegments());
  EXPECT_DOUBLE_EQ(3., timedTrajectory->getDuration());

  Eigen::VectorXd positions;

  // Position.
  timedTrajectory->evaluate(0., state);
  mStateSpace->logMap(state, positions);
  EXPECT_TRUE(Vector2d(1.0, 2.0).isApprox(positions));

  timedTrajectory->evaluate(1., state);
  mStateSpace->logMap(state, positions);
  EXPECT_TRUE(Vector2d(1.5, 2.5).isApprox(positions));

  timedTrajectory->evaluate(2., state);
  mStateSpace->logMap(state, positions);
  EXPECT_TRUE(Vector2d(2.5, 3.5).isApprox(positions));

  timedTrajectory->evaluate(3., state);
  mStateSpace->logMap(state, positions);
  EXPECT_TRUE(Vector2d(3.0, 4.0).isApprox(positions));

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
  // EXPECT_TRUE(Vector2d(0.0, 0.0).isApprox(tangentVector));

  // Acceleration.
  timedTrajectory->evaluateDerivative(0.5, 2, tangentVector);
  EXPECT_TRUE(Vector2d(1., 1.).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(1.5, 2, tangentVector);
  // TODO: isApprox does not work when comparing against zero.
  // EXPECT_TRUE(Vector2d(0., 0.).isApprox(tangentVector));

  timedTrajectory->evaluateDerivative(2.5, 2, tangentVector);
  EXPECT_TRUE(Vector2d(-1., -1.).isApprox(tangentVector));
}

TEST_F(ParabolicTimerTests, StraightLine_DifferentAccelerationLimits)
{
  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd tangentVector;
  Vector2d p1(1, 2), p2(3, 4);

  // The optimal timing of this trajectory should be a trapezoid that:
  // - accelerates at 1 rad/s^2 for 1 s
  // - coasts at 1 m/s for 1 s
  // - deaccelerates at -1 rad/s^2 for 1 s
  //
  // Note that the second dimension of the state space could result in a faster
  // timing by executing a triangular velocity profile. This is not possible
  // because the first dimension has a lower acceleration limit.

  mStateSpace->expMap(p1, state);
  inputTrajectory.addWaypoint(0., state);

  mStateSpace->expMap(p2, state);
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

  std::shared_ptr<GeodesicInterpolator> interpolator
      = std::make_shared<GeodesicInterpolator>(stateSpace);

  Interpolated inputTrajectory(stateSpace, interpolator);

  state.setQuaternion(Eigen::Quaterniond::Identity());
  inputTrajectory.addWaypoint(0., state);

  state.setQuaternion(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Vector3d::UnitX())));
  inputTrajectory.addWaypoint(1., state);

  EXPECT_THROW({ convertToSpline(inputTrajectory); }, std::invalid_argument);
}

TEST_F(ParabolicTimerTests, UnsupportedCartesianProduct_Throws)
{
  auto stateSpace = std::make_shared<CartesianProduct>(
      std::vector<ConstStateSpacePtr>{std::make_shared<SO3>()});
  auto state = stateSpace->createState();

  std::shared_ptr<GeodesicInterpolator> interpolator
      = std::make_shared<GeodesicInterpolator>(stateSpace);

  Interpolated inputTrajectory(stateSpace, interpolator);

  state.getSubStateHandle<SO3>(0).setQuaternion(Eigen::Quaterniond::Identity());
  inputTrajectory.addWaypoint(0., state);

  state.getSubStateHandle<SO3>(0).setQuaternion(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Vector3d::UnitX())));
  inputTrajectory.addWaypoint(1., state);

  EXPECT_THROW({ convertToSpline(inputTrajectory); }, std::invalid_argument);
}

// TODO: Test what happens when two waypoints are coincident.
// TODO: Add a test for different velocity limits.
// TODO: Add a test where DOFs have different ramp transition points.
