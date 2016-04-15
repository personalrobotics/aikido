#include <aikido/planner/SnapPlanner.hpp>
#include <gtest/gtest.h>
#include "../constraint/MockConstraints.hpp"
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/CollisionConstraint.hpp>
#include <aikido/distance/DistanceMetricDefaults.hpp>
#include <aikido/constraint/TestableConstraint.hpp>
#include <dart/dart.h>
#include <tuple>

using std::shared_ptr;
using std::make_shared;

class SnapPlannerTest : public ::testing::Test
{
public:
  using FCLCollisionDetector = dart::collision::FCLCollisionDetector;
  using StateSpace = aikido::statespace::MetaSkeletonStateSpace;
  using CollisionConstraint = aikido::constraint::CollisionConstraint;
  using DistanceMetric = aikido::distance::DistanceMetric;
  using MetaSkeletonStateSpace = aikido::statespace::MetaSkeletonStateSpace;
  using SO2StateSpace = aikido::statespace::SO2StateSpace;
  using GeodesicInterpolator = aikido::statespace::GeodesicInterpolator;
  using ScopedState = MetaSkeletonStateSpace::ScopedState;

  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using JointPtr = dart::dynamics::JointPtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;

  SnapPlannerTest()
      : skel{dart::dynamics::Skeleton::create("skel")}
      , jn_bn{skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>()}
      , stateSpace{make_shared<MetaSkeletonStateSpace>(skel)}
      , startState{make_shared<ScopedState>(stateSpace->createState())}
      , goalState{make_shared<ScopedState>(stateSpace->createState())}
      , passingConstraint{make_shared<PassingConstraint>(stateSpace)}
      , failingConstraint{make_shared<FailingConstraint>(stateSpace)}
      , interpolator(make_shared<GeodesicInterpolator>(stateSpace))
  {
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
  aikido::planner::PlanningResult planningResult;
};

TEST_F(SnapPlannerTest, ThrowsOnStateSpaceMismatch)
{
  SkeletonPtr empty_skel = dart::dynamics::Skeleton::create("skel");
  auto differentStateSpace = make_shared<MetaSkeletonStateSpace>(empty_skel);
  EXPECT_THROW(planSnap(*startState, *goalState, differentStateSpace,
                        passingConstraint, interpolator, &planningResult),
               std::invalid_argument);
}

TEST_F(SnapPlannerTest, ReturnsStartToGoalTrajOnSuccess)
{
  stateSpace->getStateFromMetaSkeleton(*startState);
  stateSpace->getMetaSkeleton()->setPosition(0, 2.0);
  stateSpace->getStateFromMetaSkeleton(*goalState);

  auto traj = planSnap(*startState, *goalState, stateSpace, passingConstraint,
                       interpolator, &planningResult);

  auto subSpace = stateSpace->getSubSpace<SO2StateSpace>(0);
  EXPECT_EQ(2, traj.size());

  auto startValue =
      startState->getSubStateHandle<SO2StateSpace>(0).getRotation();

  auto traj0 = stateSpace->getSubStateHandle<SO2StateSpace>(
                               static_cast<MetaSkeletonStateSpace::State*>(
                                   traj.at(0).getState()),
                               0).getRotation();

  EXPECT_TRUE(startValue.isApprox(traj0));

  auto goalValue = goalState->getSubStateHandle<SO2StateSpace>(0).getRotation();

  auto traj1 = stateSpace->getSubStateHandle<SO2StateSpace>(
                               static_cast<MetaSkeletonStateSpace::State*>(
                                   traj.at(traj.size() - 1).getState()),
                               0).getRotation();

  EXPECT_TRUE(goalValue.isApprox(traj1))
      << "on success final element of trajectory should be goal state.";
}

TEST_F(SnapPlannerTest, FailIfConstraintNotSatisfied)
{
  auto traj = planSnap(*startState, *goalState, stateSpace, failingConstraint,
                       interpolator, &planningResult);
  EXPECT_EQ(0, traj.size());  // TODO boost::optional
}
