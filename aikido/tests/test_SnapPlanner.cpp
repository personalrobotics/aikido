#include <gtest/gtest.h>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/CollisionConstraint.hpp>
#include <dart/dart.h>
#include <tuple>

using std::shared_ptr;
using std::make_shared;

class SnapPlannerTest : public ::testing::Test
{
public:
  using FCLCollisionDetector = dart::collision::FCLCollisionDetector;
  using MetaSkeletonStateSpace = aikido::statespace::MetaSkeletonStateSpace;
  using RealVectorStateSpace = aikido::statespace::RealVectorStateSpace;
  using ScopedState = MetaSkeletonStateSpace::ScopedState;
  using CollisionConstraint = aikido::constraint::CollisionConstraint;

  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using JointPtr = dart::dynamics::JointPtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;

  SnapPlannerTest()
      : skel{dart::dynamics::Skeleton::create("skel")}
      , jn_bn{skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>()}
      , cd{FCLCollisionDetector::create()}
    {
      // set joint limit to make a RealVectorStateSpace
      // instead of an SO2StateSpace
      jn_bn.first->setPositionUpperLimit(0, 3);
      stateSpace.reset(new MetaSkeletonStateSpace(skel));
      startState.reset(new ScopedState(stateSpace->createState()));
      goalState.reset(new ScopedState(stateSpace->createState()));
      collConstraint.reset(new CollisionConstraint(stateSpace, cd));
    };
  // DART setup
  SkeletonPtr skel;
  std::pair<JointPtr, BodyNodePtr> jn_bn;

  // Arguments for planner
  shared_ptr<MetaSkeletonStateSpace> stateSpace;
  shared_ptr<ScopedState> startState;
  shared_ptr<ScopedState> goalState;
  shared_ptr<FCLCollisionDetector> cd;
  shared_ptr<CollisionConstraint> collConstraint;
  aikido::planner::PlanningResult planningResult;
};

TEST_F(SnapPlannerTest, ThrowsOnStateSpaceMismatch)
{
  SkeletonPtr empty_skel = dart::dynamics::Skeleton::create("skel");
  auto differentStateSpace = make_shared<MetaSkeletonStateSpace>(empty_skel);
  EXPECT_THROW(planSnap(*startState, *goalState, differentStateSpace,
                        collConstraint, &planningResult),
               std::invalid_argument);
}

TEST_F(SnapPlannerTest, ReturnsStartGoalTrajOnSuccess)
{
  stateSpace->getStateFromMetaSkeleton(*startState);
  stateSpace->getMetaSkeleton()->setPosition(0, 10.0);
  stateSpace->getStateFromMetaSkeleton(*goalState);

  auto traj = planSnap(*startState, *goalState, stateSpace, collConstraint,
                       &planningResult);

  auto subSpace = stateSpace->getSubSpace<RealVectorStateSpace>(0);
  EXPECT_EQ(2, traj->size());

  auto startValue =
      startState->getSubStateHandle<RealVectorStateSpace>(0).getValue();

  auto traj0 = stateSpace->getSubStateHandle<RealVectorStateSpace>(
    static_cast<MetaSkeletonStateSpace::State*>(traj->at(0).getState()), 0)
      .getValue();

  EXPECT_TRUE(startValue.isApprox(traj0));
}
