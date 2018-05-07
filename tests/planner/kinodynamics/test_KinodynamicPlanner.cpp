#include <tuple>
#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/Testable.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/kinodynamics/KinodynamicPlanner.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include "../../constraint/MockConstraints.hpp"

using std::shared_ptr;
using std::make_shared;
using aikido::planner::kinodynamics::planMinimumTimeViaConstraint;

class KinodynamicPlannerTest : public ::testing::Test
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

  KinodynamicPlannerTest()
    : skel{dart::dynamics::Skeleton::create("skel")}
    , jn_bn{skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>()}
    , stateSpace{make_shared<MetaSkeletonStateSpace>(skel.get())}
    , passingConstraint{make_shared<PassingConstraint>(stateSpace)}
    , failingConstraint{make_shared<FailingConstraint>(stateSpace)}
    , interpolator(make_shared<GeodesicInterpolator>(stateSpace))
    , maxPlanTime(30.)
    , maxDistanceBtwValidityChecks(0.01)
  {
  }

  // DART setup
  SkeletonPtr skel;
  std::pair<JointPtr, BodyNodePtr> jn_bn;

  // Arguments for planner
  shared_ptr<MetaSkeletonStateSpace> stateSpace;

  Eigen::VectorXd viaVelocity;
  shared_ptr<PassingConstraint> passingConstraint;
  shared_ptr<FailingConstraint> failingConstraint;
  shared_ptr<GeodesicInterpolator> interpolator;
  aikido::planner::PlanningResult planningResult;

  double maxPlanTime;
  double maxDistanceBtwValidityChecks;
};


TEST_F(KinodynamicPlannerTest, ReturnsStartToGoalTrajOnSuccess)
{
  Eigen::Vector2d startPose(0, 2.0);
  Eigen::Vector2d viaPose(0, 1.1);
  Eigen::Vector2d goalPose(0, 0);

  aikido::statespace::StateSpace::State* startState = stateSpace->createState();
  aikido::statespace::StateSpace::State* viaState = stateSpace->createState();
  aikido::statespace::StateSpace::State* goalState = stateSpace->createState();
  stateSpace->expMap(startPose, startState);
  stateSpace->expMap(viaPose, viaState);
  stateSpace->expMap(goalPose, goalState);

  auto traj = planMinimumTimeViaConstraint(
      startState,
      goalState,
      viaState,
      viaVelocity,
      skel,
      stateSpace,
      passingConstraint,
      maxPlanTime,
      maxDistanceBtwValidityChecks);
}

TEST_F(KinodynamicPlannerTest, FailIfConstraintNotSatisfied)
{
  Eigen::Vector2d startPose(0, 2.0);
  Eigen::Vector2d viaPose(0, 1.1);
  Eigen::Vector2d goalPose(0, 0);

  aikido::statespace::StateSpace::State* startState = stateSpace->createState();
  aikido::statespace::StateSpace::State* viaState = stateSpace->createState();
  aikido::statespace::StateSpace::State* goalState = stateSpace->createState();
  stateSpace->expMap(startPose, startState);
  stateSpace->expMap(viaPose, viaState);
  stateSpace->expMap(goalPose, goalState);

  auto traj = planMinimumTimeViaConstraint(
      startState,
      goalState,
      viaState,
      viaVelocity,
      skel,
      stateSpace,
      passingConstraint,
      maxPlanTime,
      maxDistanceBtwValidityChecks);
  EXPECT_EQ(nullptr, traj);
}
