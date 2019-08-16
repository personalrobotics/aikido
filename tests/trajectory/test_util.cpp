#include <tuple>
#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/io/yaml.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/util.hpp>
#include "../constraint/MockConstraints.hpp"

using std::shared_ptr;
using std::make_shared;
using aikido::trajectory::Interpolated;
using aikido::statespace::R1;
using aikido::planner::ConfigurationToConfiguration;
using aikido::planner::SnapConfigurationToConfigurationPlanner;
using aikido::statespace::CartesianProduct;
using aikido::statespace::ConstStateSpacePtr;
using aikido::trajectory::convertToSpline;
using aikido::trajectory::loadSplineTrajectory;
using aikido::trajectory::saveTrajectory;
using aikido::trajectory::toR1JointTrajectory;

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
  auto convertedTrajectory = toR1JointTrajectory(*(trajectory.get()));

  // // Test the states in the interpolated trajectory.
  std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
  for (std::size_t i = 0; i < stateSpace->getDimension(); ++i)
    subspaces.emplace_back(std::make_shared<const R1>());

  auto rSpace = std::make_shared<CartesianProduct>(subspaces);
  auto rInterpolator = std::make_shared<GeodesicInterpolator>(rSpace);
  auto rTrajectory = std::make_shared<Interpolated>(rSpace, rInterpolator);

  EXPECT_EQ(3, convertedTrajectory->getNumWaypoints());

  Eigen::VectorXd testVector(convertedTrajectory->getNumWaypoints());
  testVector << 3.0, M_PI, 2 * M_PI - 3.0;
  for (std::size_t i = 0; i < convertedTrajectory->getNumWaypoints(); ++i)
  {
    auto sstate = convertedTrajectory->getWaypoint(i);
    stateSpace->logMap(sstate, stateVector);
    EXPECT_EQ(stateVector(0), testVector(i));
  }
  auto sstate = stateSpace->createState();
  convertedTrajectory->evaluate(
      convertedTrajectory->getDuration() * 0.75, sstate);
  stateSpace->logMap(sstate, stateVector);
  EXPECT_EQ(stateVector(0), (testVector(2) + testVector(1)) / 2.0);
}

//==============================================================================
class SaveLoadTrajectoryTest : public ::testing::Test
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

  SaveLoadTrajectoryTest()
    : skel{dart::dynamics::Skeleton::create("skel")}
    , jn_bn{skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>()}
    , stateSpace{make_shared<MetaSkeletonStateSpace>(skel.get())}
    , startState{make_shared<ScopedState>(stateSpace->createState())}
    , goalState{make_shared<ScopedState>(stateSpace->createState())}
    , passingConstraint{make_shared<PassingConstraint>(stateSpace)}
    , interpolator(make_shared<GeodesicInterpolator>(stateSpace))
    , interpolated(make_shared<Interpolated>(stateSpace, interpolator))
  {
    // Do nothing
  }

  void SetUp() override
  {
    stateSpace->getState(skel.get(), *startState);
    skel->setPosition(0, 2.0);
    stateSpace->setState(skel.get(), *goalState);

    auto problem = ConfigurationToConfiguration(
        stateSpace, *startState, *goalState, passingConstraint);
    auto planner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
        stateSpace, interpolator);
    auto traj = planner->plan(problem, &planningResult);
  }

  ~SaveLoadTrajectoryTest()
  {
    std::remove("test.yml");
  }

  SkeletonPtr skel;
  std::pair<JointPtr, BodyNodePtr> jn_bn;

  shared_ptr<MetaSkeletonStateSpace> stateSpace;
  shared_ptr<ScopedState> startState;
  shared_ptr<ScopedState> goalState;
  shared_ptr<PassingConstraint> passingConstraint;
  shared_ptr<GeodesicInterpolator> interpolator;
  shared_ptr<Interpolated> interpolated;
  SnapConfigurationToConfigurationPlanner::Result planningResult;
};

//==============================================================================
TEST_F(SaveLoadTrajectoryTest, SavedMatchesLoaded)
{
  auto testable = std::make_shared<aikido::constraint::Satisfied>(stateSpace);

  auto originalSmoothTrajectory = convertToSpline(*interpolated);
  saveTrajectory(*originalSmoothTrajectory, "test.yml");

  auto loadedSmoothTrajectory = loadSplineTrajectory("test.yml", stateSpace);

  EXPECT_EQ(originalSmoothTrajectory->getDuration(),
      loadedSmoothTrajectory->getDuration());
  for (std::size_t i = 0; i < loadedSmoothTrajectory->getNumSegments(); ++i)
  {
    auto segment_id = "seg_" + std::to_string(i);
    Eigen::MatrixXd originalCoefficients
        = originalSmoothTrajectory->getSegmentCoefficients(i);
    Eigen::MatrixXd loadedCoefficients
        = loadedSmoothTrajectory->getSegmentCoefficients(i);
    EXPECT_TRUE(loadedCoefficients.isApprox(originalCoefficients));

    Eigen::VectorXd originalPosition(stateSpace->getDimension());
    Eigen::VectorXd loadedPosition(stateSpace->getDimension());
    stateSpace->logMap(
        originalSmoothTrajectory->getSegmentStartState(i), originalPosition);
    stateSpace->logMap(
        loadedSmoothTrajectory->getSegmentStartState(i), loadedPosition);
    EXPECT_TRUE(loadedPosition.isApprox(originalPosition));
  }
}
