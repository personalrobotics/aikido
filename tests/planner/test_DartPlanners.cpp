#include <tuple>
#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/Testable.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/ConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/dart/ConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToTSR.hpp>
#include <aikido/planner/dart/DartPlannerAdapter.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include "../constraint/MockConstraints.hpp"

using std::shared_ptr;
using std::make_shared;
using aikido::trajectory::Interpolated;
using aikido::planner::ConfigurationToConfiguration;
using aikido::planner::SnapConfigurationToConfigurationPlanner;

using aikido::planner::dart::DartPlannerAdapter;
using ConfigurationToConfigurationPlanner
    = aikido::planner::ConfigurationToConfigurationPlanner;
using DartConfigurationToConfigurationPlanner
    = aikido::planner::dart::ConfigurationToConfigurationPlanner;
using aikido::planner::dart::ConfigurationToConfiguration_to_ConfigurationToTSR;

//==============================================================================
class SnapPlannerTest : public ::testing::Test
{
public:
  using FCLCollisionDetector = dart::collision::FCLCollisionDetector;
  using DistanceMetric = aikido::distance::DistanceMetric;
  using MetaSkeletonStateSpace
      = aikido::statespace::dart::MetaSkeletonStateSpace;
  using SO2 = aikido::statespace::SO2;
  using GeodesicInterpolator = aikido::statespace::GeodesicInterpolator;
  using ScopedState = MetaSkeletonStateSpace::ScopedState;

  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using JointPtr = dart::dynamics::JointPtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;

  SnapPlannerTest()
    : skel{dart::dynamics::Skeleton::create("skel")}
    , jn_bn{skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>()}
    , stateSpace{make_shared<MetaSkeletonStateSpace>(skel.get())}
    , startState{make_shared<ScopedState>(stateSpace->createState())}
    , goalState{make_shared<ScopedState>(stateSpace->createState())}
    , passingConstraint{make_shared<PassingConstraint>(stateSpace)}
    , failingConstraint{make_shared<FailingConstraint>(stateSpace)}
    , interpolator(make_shared<GeodesicInterpolator>(stateSpace))
  {
    // Do nothing
  }

  // DART setup
  SkeletonPtr skel;
  std::pair<JointPtr, BodyNodePtr> jn_bn;

  // Arguments for planner
  shared_ptr<MetaSkeletonStateSpace> stateSpace;
  shared_ptr<ScopedState> startState;
  shared_ptr<ScopedState> goalState;
  shared_ptr<PassingConstraint> passingConstraint;
  shared_ptr<FailingConstraint> failingConstraint;
  shared_ptr<GeodesicInterpolator> interpolator;
  SnapConfigurationToConfigurationPlanner::Result planningResult;
};

//==============================================================================
class UnknownProblem : public aikido::planner::Problem
{
public:
  UnknownProblem()
    : aikido::planner::Problem(std::make_shared<aikido::statespace::SO2>())
  {
    // Do nothing
  }

  const std::string& getType() const override
  {
    return getStaticType();
  }

  static const std::string& getStaticType()
  {
    static std::string name("UnknownProblem");
    return name;
  }
};

//==============================================================================
TEST_F(SnapPlannerTest, DartConfigurationToConfigurationPlanner)
{
  auto problem = ConfigurationToConfiguration(
      stateSpace, *startState, *goalState, passingConstraint);
  auto delegate = std::make_shared<SnapConfigurationToConfigurationPlanner>(
      stateSpace, interpolator);
  auto planner = std::
      make_shared<DartPlannerAdapter<SnapConfigurationToConfigurationPlanner,
                                     DartConfigurationToConfigurationPlanner>>(
          delegate, skel);
  auto traj = planner->plan(problem, &planningResult);

  if (auto interpolated = std::dynamic_pointer_cast<Interpolated>(traj))
  {
    EXPECT_EQ(2, interpolated->getNumWaypoints());
  }

  auto startValue = startState->getSubStateHandle<SO2>(0).toRotation();

  auto tmpState = stateSpace->createState();
  traj->evaluate(0, tmpState);
  auto traj0 = stateSpace->getSubStateHandle<SO2>(tmpState, 0).toRotation();

  EXPECT_TRUE(startValue.isApprox(traj0));

  auto goalValue = goalState->getSubStateHandle<SO2>(0).toRotation();

  traj->evaluate(traj->getDuration(), tmpState);
  auto traj1 = stateSpace->getSubStateHandle<SO2>(tmpState, 0).toRotation();

  EXPECT_TRUE(goalValue.isApprox(traj1))
      << "on success final element of trajectory should be goal state.";
}
