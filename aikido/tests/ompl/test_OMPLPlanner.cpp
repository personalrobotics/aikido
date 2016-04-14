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

static std::unique_ptr<DefaultRNG> make_rng()
{
  return make_unique<RNGWrapper<std::default_random_engine>>(0);
}

TEST(OMPLPlannerTest, Plan)
{
  auto skel = dart::dynamics::Skeleton::create("robot");

  auto jn_bn =
      skel->createJointAndBodyNodePair<dart::dynamics::TranslationalJoint>();

  // TODO: Set bounds on the skeleton
  skel->setPositionLowerLimit(0, -5);
  skel->setPositionUpperLimit(0, 5);
  skel->setPositionLowerLimit(1, -5);
  skel->setPositionUpperLimit(1, 5);
  skel->setPositionLowerLimit(2, 0);
  skel->setPositionUpperLimit(2, 0);

  // State space for this robot
  using StateSpace = aikido::statespace::MetaSkeletonStateSpace;
  auto stateSpace = make_shared<StateSpace>(skel);

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

  // Sampler
  std::unique_ptr<aikido::constraint::SampleableConstraint> sampler =
      aikido::constraint::createSampleableBounds(stateSpace, make_rng());

  // Joint limits
  std::unique_ptr<aikido::constraint::TestableConstraint> jlimit =
      aikido::constraint::createTestableBounds(stateSpace);

  // Plan
  aikido::path::TrajectoryPtr traj =
      aikido::ompl::planOMPL<ompl::geometric::RRTConnect>(
          startState, goalState, stateSpace, collConstraint, std::move(jlimit),
          dmetric, std::move(sampler), 5.0);

  // Check the first waypoint
  auto s0 = stateSpace->createState();
  traj->evaluate(0, s0);
  auto r0 = s0.getSubStateHandle<RealVectorStateSpace>(0);
  ASSERT_TRUE(r0.getValue().isApprox(start_pose));

  // Check the last waypoint
  traj->evaluate(traj->getDuration(), s0);
  r0 = s0.getSubStateHandle<RealVectorStateSpace>(0);
  ASSERT_TRUE(r0.getValue().isApprox(goal_pose));


  
}
