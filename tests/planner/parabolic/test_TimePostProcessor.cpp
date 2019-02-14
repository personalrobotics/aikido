#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/trajectory/util.hpp>
#include "eigen_tests.hpp"

using Eigen::Vector2d;
using aikido::trajectory::Interpolated;
using aikido::statespace::CartesianProduct;
using aikido::statespace::ConstStateSpacePtr;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R1;
using aikido::trajectory::convertToSpline;
using aikido::planner::parabolic::ParabolicTimer;

class TimePostProcessorTests : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mRng = aikido::common::RNGWrapper<std::mt19937>(0);
    std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
    for (std::size_t i = 0; i < 2; ++i)
      subspaces.emplace_back(std::make_shared<R1>());
    mStateSpace = std::make_shared<CartesianProduct>(subspaces);
    mInterpolator = std::make_shared<GeodesicInterpolator>(mStateSpace);
  }

  aikido::common::RNGWrapper<std::mt19937> mRng;
  std::shared_ptr<CartesianProduct> mStateSpace;
  std::shared_ptr<GeodesicInterpolator> mInterpolator;
};

TEST_F(TimePostProcessorTests, testTime)
{
  ParabolicTimer testTimePostProcessor(
      Vector2d::Constant(2.), Vector2d::Constant(1.));

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

  auto timedTrajectory
      = testTimePostProcessor.postprocess(inputTrajectory, mRng);

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

TEST_F(TimePostProcessorTests, testSplineTiming)
{
  ParabolicTimer testTimePostProcessor(
      Vector2d::Constant(2.), Vector2d::Constant(1.));

  Interpolated interpolated(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  auto state2 = mStateSpace->createState();
  Eigen::VectorXd positions, positions2;
  Vector2d p1(1, 2), p2(2, 3);

  // This is the same test as StraightLine_TriangularProfile, except that the
  // trajectory starts at a non-zero time.
  mStateSpace->expMap(p1, state);
  interpolated.addWaypoint(1., state);

  mStateSpace->expMap(p2, state);
  interpolated.addWaypoint(3., state);

  auto spline = convertToSpline(interpolated);

  auto timedInterpolated
      = testTimePostProcessor.postprocess(interpolated, mRng);
  auto timedSpline = testTimePostProcessor.postprocess(*spline, mRng);

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
