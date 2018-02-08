#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include "eigen_tests.hpp"

using Eigen::Vector2d;
using Eigen::Vector3d;
using aikido::trajectory::Interpolated;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R2;
using aikido::statespace::StateSpacePtr;
using aikido::constraint::Satisfied;
using aikido::common::cloneRNGFrom;
using aikido::planner::parabolic::convertToSpline;
using aikido::planner::parabolic::ParabolicTimer;

class TimePostProcessorTests : public ::testing::Test
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  void SetUp() override
  {
    mRng = aikido::common::RNGWrapper<std::mt19937>(0);
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

  aikido::common::RNGWrapper<std::mt19937> mRng;
  std::shared_ptr<R2> mStateSpace;
  Eigen::Vector2d mMaxVelocity;
  Eigen::Vector2d mMaxAcceleration;

  std::shared_ptr<GeodesicInterpolator> mInterpolator;
  std::shared_ptr<Interpolated> mStraightLine;
};

TEST_F(TimePostProcessorTests, testTime)
{
  ParabolicTimer testTimePostProcessor(
      Vector2d::Constant(2.), Vector2d::Constant(1.));

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
      = testTimePostProcessor.postprocess(inputTrajectory, mRng);

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

TEST_F(TimePostProcessorTests, testSplineTiming)
{
  ParabolicTimer testTimePostProcessor(
      Vector2d::Constant(2.), Vector2d::Constant(1.));

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
      = testTimePostProcessor.postprocess(interpolated, mRng);
  auto timedSpline = testTimePostProcessor.postprocess(*spline, mRng);

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
