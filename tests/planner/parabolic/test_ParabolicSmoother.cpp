#include <gtest/gtest.h>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/SO3.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>

using Eigen::Vector2d;
using Eigen::Vector3d;
using aikido::trajectory::Interpolated;
using aikido::planner::parabolic::computeParabolicTiming;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R2;
using aikido::statespace::CartesianProduct;
using aikido::statespace::SO2;
using aikido::statespace::SO3;
using aikido::statespace::StateSpacePtr;

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
