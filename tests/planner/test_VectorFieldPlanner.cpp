#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <gtest/gtest.h>
#include "../constraint/MockConstraints.hpp"
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/NonColliding.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/constraint/Testable.hpp>
#include <dart/dart.hpp>
#include <tuple>

using std::shared_ptr;
using std::make_shared;

class VectorFieldPlannerTest : public ::testing::Test
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

  VectorFieldPlannerTest()
      : skel{dart::dynamics::Skeleton::create("skel")}
      , jn_bn{skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>()}
      , stateSpace{std::make_shared<MetaSkeletonStateSpace>(skel)}
      , startState{std::make_shared<ScopedState>(stateSpace->createState())}
      , goalState{std::make_shared<ScopedState>(stateSpace->createState())}
      , passingConstraint{std::make_shared<PassingConstraint>(stateSpace)}
      , failingConstraint{std::make_shared<FailingConstraint>(stateSpace)}
      , interpolator(std::make_shared<GeodesicInterpolator>(stateSpace))
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

TEST_F(VectorFieldPlannerTest, PlanToEndEffectorOffsetTest)
{


}

TEST_F(VectorFieldPlannerTest, PlanToEndEffectorOffsetCollisionTest)
{


}
