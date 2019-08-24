#include "aikido/io/trajectory.hpp"

#include <tuple>
#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/io/yaml.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/trajectory/util.hpp>
#include "../constraint/MockConstraints.hpp"

using std::shared_ptr;
using std::make_shared;
using aikido::io::loadSplineTrajectory;
using aikido::io::saveTrajectory;
using aikido::trajectory::Interpolated;
using aikido::planner::ConfigurationToConfiguration;
using aikido::planner::SnapConfigurationToConfigurationPlanner;
using aikido::statespace::ConstStateSpacePtr;
using aikido::trajectory::convertToSpline;

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
    interpolated = std::dynamic_pointer_cast<Interpolated>(traj);
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
    Eigen::MatrixXd originalCoefficients
        = originalSmoothTrajectory->getSegmentCoefficients(i);
    Eigen::MatrixXd loadedCoefficients
        = loadedSmoothTrajectory->getSegmentCoefficients(i);
    EXPECT_TRUE(loadedCoefficients.isApprox(originalCoefficients)) << loadedCoefficients;

    Eigen::VectorXd originalPosition(stateSpace->getDimension());
    Eigen::VectorXd loadedPosition(stateSpace->getDimension());
    stateSpace->logMap(
        originalSmoothTrajectory->getSegmentStartState(i), originalPosition);
    stateSpace->logMap(
        loadedSmoothTrajectory->getSegmentStartState(i), loadedPosition);
    EXPECT_TRUE(loadedPosition.isApprox(originalPosition));
  }
}
