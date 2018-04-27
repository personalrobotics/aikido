#include <tuple>
#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/Testable.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
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
  UnknownProblem() : aikido::planner::Problem(nullptr)
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
TEST_F(SnapPlannerTest, CanSolveProblems)
{
  auto planner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
      stateSpace, interpolator);

  auto problem = ConfigurationToConfiguration(
      stateSpace, *startState, *goalState, failingConstraint);
  auto unknownProblem = UnknownProblem();

  EXPECT_TRUE(planner->canSolve(problem));
  EXPECT_FALSE(planner->canSolve(unknownProblem));
}

//==============================================================================
TEST_F(SnapPlannerTest, ThrowsOnStateSpaceMismatch)
{
  SkeletonPtr emptySkel = dart::dynamics::Skeleton::create("skel");
  auto differentStateSpace
      = make_shared<MetaSkeletonStateSpace>(emptySkel.get());
  EXPECT_THROW(
      {
        auto problem = ConfigurationToConfiguration(
            differentStateSpace, *startState, *goalState, passingConstraint);
        DART_UNUSED(problem);
      },
      std::invalid_argument);
}

//==============================================================================
TEST_F(SnapPlannerTest, ReturnsStartToGoalTrajOnSuccess)
{
  stateSpace->getState(skel.get(), *startState);
  skel->setPosition(0, 2.0);
  stateSpace->setState(skel.get(), *goalState);

  auto problem = ConfigurationToConfiguration(
      stateSpace, *startState, *goalState, passingConstraint);
  auto planner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
      stateSpace, interpolator);
  auto traj = planner->plan(problem, &planningResult);

  auto subSpace = stateSpace->getSubspace<SO2>(0);
  DART_UNUSED(subSpace);

  if (auto interpolated = std::dynamic_pointer_cast<Interpolated>(traj))
  {
    EXPECT_EQ(2, interpolated->getNumWaypoints());
  }

  auto startValue = startState->getSubStateHandle<SO2>(0).getRotation();

  auto tmpState = stateSpace->createState();
  traj->evaluate(0, tmpState);
  auto traj0 = stateSpace->getSubStateHandle<SO2>(tmpState, 0).getRotation();

  EXPECT_TRUE(startValue.isApprox(traj0));

  auto goalValue = goalState->getSubStateHandle<SO2>(0).getRotation();

  traj->evaluate(traj->getDuration(), tmpState);
  auto traj1 = stateSpace->getSubStateHandle<SO2>(tmpState, 0).getRotation();

  EXPECT_TRUE(goalValue.isApprox(traj1))
      << "on success final element of trajectory should be goal state.";
}

//==============================================================================
TEST_F(SnapPlannerTest, FailIfConstraintNotSatisfied)
{
  auto problem = ConfigurationToConfiguration(
      stateSpace, *startState, *goalState, failingConstraint);
  auto planner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
      stateSpace, interpolator);
  auto traj = planner->plan(problem, &planningResult);
  EXPECT_EQ(nullptr, traj);
}
