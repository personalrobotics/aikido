#include <gtest/gtest.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/CollisionConstraint.hpp>
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

static std::unique_ptr<DefaultRNG> make_rng() {
  return make_unique<RNGWrapper<std::default_random_engine>>(0);
}

TEST(OMPLPlannerTest, Plan) {
  auto skel = dart::dynamics::Skeleton::create("robot");

  auto jn_bn =
      skel->createJointAndBodyNodePair<dart::dynamics::TranslationalJoint>();

  // State space for this robot
  using StateSpace = aikido::statespace::MetaSkeletonStateSpace;
  auto stateSpace = make_shared<StateSpace>(skel);

  // Set bounds on the state space

  // Start state - 3 dof real vector
  using RealVectorStateSpace = aikido::statespace::RealVectorStateSpace;
  StateSpace::ScopedState startState = stateSpace->createState();
  auto subState = startState.getSubStateHandle<RealVectorStateSpace>(0);
  Eigen::Vector3d start_pose(-5, -5, 0);
  subState.setValue(start_pose);

  stateSpace->setStateOnMetaSkeleton(startState);

  // Goal state
  StateSpace::ScopedState goalState = stateSpace->createState();
  Eigen::Vector3d goal_pose(5, 5, 0);
  subState = goalState.getSubStateHandle<RealVectorStateSpace>(0);
  subState.setValue(goal_pose);

  // Collision constraint
  auto cd = dart::collision::FCLCollisionDetector::create();
  auto collConstraint =
      std::make_shared<aikido::constraint::CollisionConstraint>(stateSpace, cd);

  // Distance metric
  std::shared_ptr<aikido::statespace::CompoundStateSpace> cspace = stateSpace;
  auto dmetric = aikido::distance::createDistanceMetricFor(cspace);

  // Plan
  aikido::ompl::planOMPL<ompl::geometric::RRTConnect>(
      startState, goalState, stateSpace, collConstraint, dmetric, 5.0,
      make_rng());
}
