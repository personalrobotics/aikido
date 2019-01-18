#include <tuple>
#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/util.hpp>


using std::shared_ptr;
using std::make_shared;
using aikido::trajectory::Interpolated;
using aikido::statespace::R1;
using aikido::statespace::CartesianProduct;
using aikido::trajectory::toR1JointTrajectory;
using aikido::statespace::ConstStateSpacePtr;

//==============================================================================
class TrajectoryConversionTest : public ::testing::Test
{
public:
  using MetaSkeletonStateSpace
      = aikido::statespace::dart::MetaSkeletonStateSpace;
  using SO2 = aikido::statespace::SO2;
  using GeodesicInterpolator = aikido::statespace::GeodesicInterpolator;
  using ScopedState = MetaSkeletonStateSpace::ScopedState;

  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using JointPtr = dart::dynamics::JointPtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;

  TrajectoryConversionTest()
    : skel{dart::dynamics::Skeleton::create("skel")}
    , jn_bn{skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>()}
    , stateSpace{make_shared<MetaSkeletonStateSpace>(skel.get())}
    , interpolator(make_shared<GeodesicInterpolator>(stateSpace))
  {
    // Do nothing
  }

  // DART setup
  SkeletonPtr skel;
  std::pair<JointPtr, BodyNodePtr> jn_bn;

  // Arguments for planner
  shared_ptr<MetaSkeletonStateSpace> stateSpace;
  shared_ptr<GeodesicInterpolator> interpolator;
};

//==============================================================================
TEST_F(TrajectoryConversionTest, SuccessfulConversionToR1)
{
  // Create Trajectory.
  auto trajectory = std::make_shared<Interpolated>(stateSpace, interpolator);

  // Add waypoints
  Eigen::VectorXd stateVector(stateSpace->getDimension());
  
  stateVector << 3.0;
  auto s1 = stateSpace->createState();
  stateSpace->convertPositionsToState(stateVector, s1);
  trajectory->addWaypoint(0, s1);

  stateVector << M_PI;
  auto s2 = stateSpace->createState();
  stateSpace->convertPositionsToState(stateVector, s2);
  trajectory->addWaypoint(1, s2);

  stateVector << -3.0;
  auto s3 = stateSpace->createState();
  stateSpace->convertPositionsToState(stateVector, s3);
  trajectory->addWaypoint(2, s3);

  // Convert the trajectory.
  ConstStateSpacePtr outputStateSpace;
  outputStateSpace = std::move(stateSpace);
  auto convertedTrajectory = toR1JointTrajectory(outputStateSpace, *(trajectory.get()));

  // // Test the states in the interpolated trajectory.
  std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
  for (std::size_t i = 0; i < outputStateSpace->getDimension(); ++i)
    subspaces.emplace_back(std::make_shared<const R1>());

  auto rSpace = std::make_shared<CartesianProduct>(subspaces);
  auto rInterpolator = std::make_shared<GeodesicInterpolator>(rSpace);
  auto rTrajectory = std::make_shared<Interpolated>(rSpace, rInterpolator);

  EXPECT_EQ(3, convertedTrajectory->getNumWaypoints());

  Eigen::VectorXd testVector(convertedTrajectory->getNumWaypoints());
  testVector << 3.0, M_PI, 2*M_PI - 3.0;
  for (std::size_t i = 0; i < convertedTrajectory->getNumWaypoints(); ++i)
  {
    auto sstate = convertedTrajectory->getWaypoint(i);
    outputStateSpace->logMap(sstate, stateVector);
    EXPECT_EQ(stateVector(0), testVector(i));
  }
}
