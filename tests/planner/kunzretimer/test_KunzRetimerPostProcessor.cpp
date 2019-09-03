#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/planner/kunzretimer/KunzRetimer.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/trajectory/util.hpp>
#include "eigen_tests.hpp"

using aikido::planner::kunzretimer::KunzRetimer;
using aikido::statespace::CartesianProduct;
using aikido::statespace::ConstStateSpacePtr;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R1;
using aikido::trajectory::convertToSpline;
using aikido::trajectory::Interpolated;
using Eigen::Vector2d;

class KunzRetimerPostProcessorTests : public ::testing::Test
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

    // set default parameters
    mMaxDeviation = 1e-1;
    mTimeStep = 0.01;
  }

  aikido::common::RNGWrapper<std::mt19937> mRng;
  std::shared_ptr<CartesianProduct> mStateSpace;
  std::shared_ptr<GeodesicInterpolator> mInterpolator;

  double mMaxDeviation;
  double mTimeStep;
};

TEST_F(KunzRetimerPostProcessorTests, testTime)
{
  KunzRetimer testKunzRetimerPostProcessor(
      Vector2d::Constant(2.), Vector2d::Constant(1.), mMaxDeviation, mTimeStep);

  Interpolated inputTrajectory(mStateSpace, mInterpolator);

  auto state = mStateSpace->createState();
  Eigen::VectorXd positions(2);
  Eigen::VectorXd tangentVector;

  // The optimal timing of this trajectory should be a triangle centered at t =
  // 1that accelerates at 1 rad/s^2 for 1 s, then deaccelerates at -1 rad/s^2
  // for 1 s. This corresponds to moving each axis through 2 rad.
  positions << 1, 2;
  mStateSpace->expMap(positions, state);
  inputTrajectory.addWaypoint(0., state);

  positions << 2, 3;
  mStateSpace->expMap(positions, state);
  inputTrajectory.addWaypoint(2., state);

  auto timedTrajectory
      = testKunzRetimerPostProcessor.postprocess(inputTrajectory, mRng);

  EXPECT_GE(timedTrajectory->getNumDerivatives(), 2);
  double durationTolerance = 1e-6;
  EXPECT_NEAR(2., timedTrajectory->getDuration(), durationTolerance);

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

  double accelerationPrecision = 1e-10;
  // Acceleration.
  timedTrajectory->evaluateDerivative(0.5, 2, tangentVector);
  EXPECT_TRUE(Vector2d(1., 1.).isApprox(tangentVector, accelerationPrecision));

  timedTrajectory->evaluateDerivative(1.5, 2, tangentVector);
  EXPECT_TRUE(
      Vector2d(-1., -1.).isApprox(tangentVector, accelerationPrecision));
}
