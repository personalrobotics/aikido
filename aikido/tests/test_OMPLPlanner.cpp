#include <gtest/gtest.h>
#include <aikido/statespace/MetaSkeletonStateSpace.hpp>
//#include <aikido/constraint/CollisionConstraint.hpp>
#include <dart/dart.h>
#include <tuple>

using std::shared_ptr;
using std::make_shared;

class OMPLPlannerTest: public ::testing::Test
{
public:
  using FCLCollisionDetector = dart::collision::FCLCollisionDetector;
  using StateSpace = aikido::statespace::MetaSkeletonStateSpace;
  using ScopedState = StateSpace::ScopedState;
//  using CollisionConstraint = aikido::constraint::CollisionConstraint;

  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using JointPtr = dart::dynamics::JointPtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;
    using RealVectorStateSpace = aikido::statespace::RealVectorStateSpace;
    
    virtual void SetUp() {
        // Create a robot that can translate and rotate in the plane
        skel = dart::dynamics::Skeleton::create("robot");
        jn_bn = skel->createJointAndBodyNodePair<dart::dynamics::TranslationalJoint>();
        // TODO: Add some shape

        // State space for this robot
        stateSpace = make_shared<StateSpace>(skel);

        // Start state - 3 dof real vector
        ScopedState startState = stateSpace->createState();
        auto subState = startState.getSubStateHandle<RealVectorStateSpace>(0);
        Eigen::Vector3d start_pose(-5, -5, 0);
        subState.setValue(start_pose);

        stateSpace->setStateOnMetaSkeleton(startState);

        // Goal state
        ScopedState goalState = stateSpace->createState();
        Eigen::Vector3d goal_pose(5, 5, 0);
        subState = goalState.getSubStateHandle<RealVectorStateSpace>(0);
        subState.setValue(goal_pose);
    }
    
  // DART setup
  SkeletonPtr skel;
  std::pair<JointPtr, BodyNodePtr> jn_bn;

  // Arguments for planner
  shared_ptr<StateSpace> stateSpace;
//  ScopedState startState;
//  ScopedState goalState;
  shared_ptr<FCLCollisionDetector> cd;
//  shared_ptr<CollisionConstraint> collConstraint;
//  aikido::planner::PlanningResult planningResult;
};

TEST_F(OMPLPlannerTest, Plan)
{

}
