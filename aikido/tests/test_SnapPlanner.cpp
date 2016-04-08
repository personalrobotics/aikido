#include <gtest/gtest.h>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/statespace/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/CollisionConstraint.hpp>
#include <dart/dart.h>
#include <tuple>

using std::shared_ptr;
using std::make_shared;

class SnapPlannerTest: public ::testing::Test
{
public:
  using FCLCollisionDetector = dart::collision::FCLCollisionDetector;
  using StateSpace = aikido::statespace::MetaSkeletonStateSpace;
  using ScopedState = StateSpace::ScopedState;
  using CollisionConstraint = aikido::constraint::CollisionConstraint;

  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using JointPtr = dart::dynamics::JointPtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;


  SnapPlannerTest()
    : skel{dart::dynamics::Skeleton::create("skel")}
    , jn_bn{skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>()}
    , stateSpace{make_shared<StateSpace>(skel)}
    , startState{make_shared<ScopedState>(stateSpace->createState())}
    , goalState{make_shared<ScopedState>(stateSpace->createState())}
    , cd{FCLCollisionDetector::create()}
    , collConstraint{make_shared<CollisionConstraint>(stateSpace, cd)}
  {
  };
  // DART setup
  SkeletonPtr skel;
  std::pair<JointPtr, BodyNodePtr> jn_bn;

  // Arguments for planner
  shared_ptr<StateSpace> stateSpace;
  shared_ptr<ScopedState> startState;
  shared_ptr<ScopedState> goalState;
  shared_ptr<FCLCollisionDetector> cd;
  shared_ptr<CollisionConstraint> collConstraint;
  aikido::planner::PlanningResult planningResult;
};

TEST_F(SnapPlannerTest, ThrowsOnStateSpaceMismatch)
{
  SkeletonPtr empty_skel = dart::dynamics::Skeleton::create("skel");
  auto differentStateSpace = make_shared<StateSpace>(empty_skel);
  EXPECT_THROW(
    planSnap(*startState, *goalState, differentStateSpace, collConstraint, &planningResult),
    std::invalid_argument
  );
}
