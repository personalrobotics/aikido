#include <aikido/planner/VectorFieldPlanner.hpp>
#include <gtest/gtest.h>
#include "../constraint/MockConstraints.hpp"
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/NonColliding.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/constraint/Testable.hpp>
#include <dart/dart.h>
#include <tuple>

using std::shared_ptr;
using std::make_shared;
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


class VectorFieldPlannerTest : public ::testing::Test
{
public:

  VectorFieldPlannerTest()
      : skel{dart::dynamics::Skeleton::create("skel")}
      , jn_bn{skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>()}
      , myStateSpace{make_shared<MetaSkeletonStateSpace>(skel)}
      , startState{make_shared<ScopedState>(myStateSpace->createState())}
      , goalState{make_shared<ScopedState>(myStateSpace->createState())}
      , passingConstraint{make_shared<PassingConstraint>(myStateSpace)}
      , failingConstraint{make_shared<FailingConstraint>(myStateSpace)}
      , interpolator(make_shared<GeodesicInterpolator>(myStateSpace))
  {
  }

  // DART setup
  SkeletonPtr skel;
  std::pair<JointPtr, BodyNodePtr> jn_bn;

  // Arguments for planner
  shared_ptr<MetaSkeletonStateSpace> myStateSpace;
  shared_ptr<ScopedState> startState;
  shared_ptr<ScopedState> goalState;
  shared_ptr<PassingConstraint> passingConstraint;
  shared_ptr<FailingConstraint> failingConstraint;
  shared_ptr<GeodesicInterpolator> interpolator;
  aikido::planner::PlanningResult planningResult;


};


  // FIXME: Compute the inverse, compose, and logmap once and not every call
//  struct testCallback{
//    bool operator()
bool testCallback_full( // const std::shared_ptr<aikido::statespace::StateSpace> stateSpace,
                const aikido::statespace::StateSpace::State *state, 
                double t, 
                Eigen::VectorXd *qd, 
                shared_ptr<MetaSkeletonStateSpace> myStateSpace,
                shared_ptr<ScopedState> startState,
                shared_ptr<ScopedState> goalState){
  shared_ptr<ScopedState> dqs = make_shared<ScopedState>(myStateSpace->createState());
  shared_ptr<ScopedState> startInv = make_shared<ScopedState>(myStateSpace->createState());
  myStateSpace->getInverse(*startState,*startInv);
  myStateSpace->compose(*goalState,*startInv,*dqs);
  myStateSpace->logMap(*dqs,*qd);
  *qd *=t;
  return true;    
}


TEST_F(VectorFieldPlannerTest, ThrowsOnStateSpaceMismatch)
{

  boost::function<bool ( aikido::statespace::StateSpace::State const *state, double t, Eigen::VectorXd *qd)> testCallback;

  testCallback = boost::bind(testCallback_full, _1, _2, _3, myStateSpace, startState, goalState);

  SkeletonPtr empty_skel = dart::dynamics::Skeleton::create("skel");
  auto differentStateSpace = make_shared<MetaSkeletonStateSpace>(empty_skel);
  EXPECT_THROW({
    aikido::planner::planVF(differentStateSpace, *startState, *goalState,
      interpolator, passingConstraint, planningResult,0.01,testCallback);
  }, std::invalid_argument);
}

TEST_F(VectorFieldPlannerTest, ReturnsStartToGoalTrajOnSuccess)
{
  myStateSpace->getState(*startState);
  myStateSpace->getMetaSkeleton()->setPosition(0, 2.0);
  myStateSpace->setState(*goalState);

  boost::function<bool ( aikido::statespace::StateSpace::State const *state, double t, Eigen::VectorXd *qd)> testCallback;

  testCallback = boost::bind(testCallback_full, _1, _2, _3, myStateSpace, startState, goalState);

  auto traj = aikido::planner::planVF(myStateSpace, *startState, *goalState, interpolator,
    passingConstraint, planningResult,0.01,testCallback);

  auto subSpace = myStateSpace->getSubspace<SO2>(0);
  EXPECT_EQ(2, traj->getNumWaypoints());

  auto startValue =
      startState->getSubStateHandle<SO2>(0).getRotation();

  auto tmpState = myStateSpace->createState();
  traj->evaluate(0, tmpState);
  auto traj0 =
      myStateSpace->getSubStateHandle<SO2>(tmpState, 0).getRotation();

  EXPECT_TRUE(startValue.isApprox(traj0));

  auto goalValue = goalState->getSubStateHandle<SO2>(0).getRotation();

  traj->evaluate(traj->getDuration(), tmpState);
  auto traj1 = myStateSpace->getSubStateHandle<SO2>(tmpState,0).getRotation();

  EXPECT_TRUE(goalValue.isApprox(traj1))
      << "on success final element of trajectory should be goal state.";
}

TEST_F(VectorFieldPlannerTest, FailIfConstraintNotSatisfied)
{
  boost::function<bool ( aikido::statespace::StateSpace::State const *state, double t, Eigen::VectorXd *qd)> testCallback;

  testCallback = boost::bind(testCallback_full, _1, _2, _3, myStateSpace, startState, goalState);

  auto traj = aikido::planner::planVF(myStateSpace, *startState, *goalState, interpolator,
    failingConstraint, planningResult,0.01,testCallback);
  EXPECT_EQ(0, traj->getNumWaypoints());  // TODO boost::optional
}

