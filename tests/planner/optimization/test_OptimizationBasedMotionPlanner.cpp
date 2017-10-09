#include <dart/dart.hpp>
#include <tuple>
#include <gtest/gtest.h>
#include "../../constraint/MockConstraints.hpp"
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/NonColliding.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/planner/optimization/Planner.hpp>

using std::shared_ptr;
using std::make_shared;

class OptimizationBasedMotionPlanner : public ::testing::Test
{
public:
  using FCLCollisionDetector = dart::collision::FCLCollisionDetector;
  using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
  using NonColliding = aikido::constraint::NonColliding;
  using DistanceMetric = aikido::distance::DistanceMetric;
  using MetaSkeletonStateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
  using SO2 = aikido::statespace::SO2;
  using GeodesicInterpolator = aikido::statespace::GeodesicInterpolator;
  using ScopedState = MetaSkeletonStateSpace::ScopedState;

  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using JointPtr = dart::dynamics::JointPtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;

  OptimizationBasedMotionPlanner()
      : skel{dart::dynamics::Skeleton::create("skel")}
      , jn_bn{skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>()}
      , stateSpace{make_shared<MetaSkeletonStateSpace>(skel)}
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
  aikido::planner::PlanningResult planningResult;
};

//==============================================================================
TEST_F(OptimizationBasedMotionPlanner, ThrowsOnStateSpaceMismatch)
{
  SkeletonPtr empty_skel = dart::dynamics::Skeleton::create("skel");
  auto differentStateSpace = make_shared<MetaSkeletonStateSpace>(empty_skel);
  EXPECT_THROW({
    planSnap(differentStateSpace, *startState, *goalState,
      interpolator, passingConstraint, planningResult);
  }, std::invalid_argument);
}

//==============================================================================
TEST_F(OptimizationBasedMotionPlanner, PlanToConfiguration)
{

}
