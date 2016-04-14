#include <gtest/gtest.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/CollisionConstraint.hpp>
#include <aikido/constraint/dart.hpp>
#include <aikido/distance/DistanceMetricDefaults.hpp>
#include <aikido/ompl/OMPLPlanner.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <dart/dart.h>
#include <tuple>

using std::shared_ptr;
using std::make_shared;
using aikido::util::RNGWrapper;
using aikido::util::RNG;
using dart::common::make_unique;
using DefaultRNG = RNGWrapper<std::default_random_engine>;
using RealVectorStateSpace = aikido::statespace::RealVectorStateSpace;

static std::unique_ptr<DefaultRNG> make_rng()
{
  return make_unique<RNGWrapper<std::default_random_engine>>(0);
}

class OMPLPlannerTest : public ::testing::Test
{
public:
  using StateSpace = aikido::statespace::MetaSkeletonStateSpace;

  OMPLPlannerTest()
      : skel{dart::dynamics::Skeleton::create("robot")}
      , stateSpace{make_shared<StateSpace>(skel)}
      , startState{stateSpace->createState()}
      , goalState{stateSpace->createState()}
      , startPose{Eigen::Vector3d(-5, -5, 0)}
      , goalPose{Eigen::Vector3d(5, 5, 0)}
  {
    skel = dart::dynamics::Skeleton::create("robot");

    auto jn_bn =
        skel->createJointAndBodyNodePair<dart::dynamics::TranslationalJoint>();

    // TODO: Set bounds on the skeleton
    skel->setPositionLowerLimit(0, -5);
    skel->setPositionUpperLimit(0, 5);
    skel->setPositionLowerLimit(1, -5);
    skel->setPositionUpperLimit(1, 5);
    skel->setPositionLowerLimit(2, 0);
    skel->setPositionUpperLimit(2, 0);

    // Start state - 3 dof real vector
    auto subState = startState.getSubStateHandle<RealVectorStateSpace>(0);
    subState.setValue(startPose);

    stateSpace->setStateOnMetaSkeleton(startState);

    // Goal state
    subState = goalState.getSubStateHandle<RealVectorStateSpace>(0);
    subState.setValue(goalPose);

    // Collision constraint
    auto cd = dart::collision::FCLCollisionDetector::create();
    collConstraint = std::make_shared<aikido::constraint::CollisionConstraint>(
        stateSpace, cd);

    // Distance metric
    dmetric = aikido::distance::createDistanceMetricFor(stateSpace);

    // Sampler
    sampler =
        aikido::constraint::createSampleableBounds(stateSpace, make_rng());

    // Projectable constraint
    projConstraint = aikido::constraint::createProjectableBounds(stateSpace);

    // Joint limits
    jlimit = aikido::constraint::createTestableBounds(stateSpace);
  }

  dart::dynamics::SkeletonPtr skel;
  aikido::statespace::MetaSkeletonStateSpacePtr stateSpace;
  StateSpace::ScopedState startState;
  StateSpace::ScopedState goalState;
  aikido::distance::DistanceMetricPtr dmetric;
  aikido::constraint::SampleableConstraintPtr sampler;
  aikido::constraint::ProjectablePtr projConstraint;
  aikido::constraint::TestableConstraintPtr jlimit;
  aikido::constraint::TestableConstraintPtr collConstraint;
  Eigen::Vector3d startPose;
  Eigen::Vector3d goalPose;
};

TEST_F(OMPLPlannerTest, Plan)
{
  // Plan
  auto traj = aikido::ompl::planOMPL<ompl::geometric::RRTConnect>(
      startState, goalState, stateSpace, std::move(collConstraint),
      std::move(jlimit), std::move(dmetric), std::move(sampler),
      std::move(projConstraint), 5.0);

  // Check the first waypoint
  auto s0 = stateSpace->createState();
  traj->evaluate(0, s0);
  auto r0 = s0.getSubStateHandle<RealVectorStateSpace>(0);
  ASSERT_TRUE(r0.getValue().isApprox(startPose));

  // Check the last waypoint
  traj->evaluate(traj->getDuration(), s0);
  r0 = s0.getSubStateHandle<RealVectorStateSpace>(0);
  ASSERT_TRUE(r0.getValue().isApprox(goalPose));
}
