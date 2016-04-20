#include <gtest/gtest.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/constraint/CollisionConstraint.hpp>
#include <aikido/constraint/dart.hpp>
#include <aikido/distance/DistanceMetricDefaults.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>
#include <aikido/ompl/OMPLPlanner.hpp>
#include <aikido/ompl/CRRT.hpp>
#include <aikido/util/StepSequence.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <dart/dart.h>
#include <tuple>

using std::shared_ptr;
using std::make_shared;
using aikido::util::RNGWrapper;
using aikido::statespace::GeodesicInterpolator;
using aikido::util::RNG;
using dart::common::make_unique;
using DefaultRNG = RNGWrapper<std::default_random_engine>;
using RealVectorStateSpace = aikido::statespace::RealVectorStateSpace;
using StateSpace = aikido::statespace::MetaSkeletonStateSpace;
using AIKIDOStateSpace = aikido::ompl::AIKIDOGeometricStateSpace;

static std::unique_ptr<DefaultRNG> make_rng()
{
  return make_unique<RNGWrapper<std::default_random_engine>>(0);
}

class TranslationalProjectionConstraint
    : public aikido::constraint::Projectable,
      public aikido::constraint::TestableConstraint
{
public:
  TranslationalProjectionConstraint(
      const aikido::statespace::MetaSkeletonStateSpacePtr &stateSpace)
      : mStateSpace(stateSpace)
  {
  }

  aikido::statespace::StateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  bool project(const aikido::statespace::StateSpace::State *instate,
               aikido::statespace::StateSpace::State *outstate) const override
  {
    auto inSubState =
        mStateSpace->getSubStateHandle<RealVectorStateSpace>(instate, 0);
    auto outSubState =
        mStateSpace->getSubStateHandle<RealVectorStateSpace>(outstate, 0);

    auto inval = inSubState.getValue();
    outSubState.setValue(Eigen::Vector3d(-3, inval[1], inval[2]));
  }

  bool isSatisfied(
      const aikido::statespace::StateSpace::State *state) const override
  {
    auto st = mStateSpace->getSubStateHandle<RealVectorStateSpace>(state, 0);
    auto stval = st.getValue();
    return fabs(stval[0] + 3) < 1e-7;
  }

private:
  aikido::statespace::MetaSkeletonStateSpacePtr mStateSpace;
};

class OMPLConstrainedTest : public ::testing::Test
{
public:
  virtual void SetUp()
  {
    // Create robot
    auto robot = dart::dynamics::Skeleton::create("robot");
    auto jn_bn =
        robot->createJointAndBodyNodePair<dart::dynamics::TranslationalJoint>();

    stateSpace = make_shared<StateSpace>(robot);
    interpolator = std::make_shared<GeodesicInterpolator>(stateSpace);

    // Set bounds on the skeleton
    robot->setPositionLowerLimit(0, -5);
    robot->setPositionUpperLimit(0, 5);
    robot->setPositionLowerLimit(1, -5);
    robot->setPositionUpperLimit(1, 5);
    robot->setPositionLowerLimit(2, 0);
    robot->setPositionUpperLimit(2, 0);

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
    boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);

    // Joint limits
    boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);

    // Trajectory constraint
    trajConstraint =
        std::make_shared<TranslationalProjectionConstraint>(stateSpace);
  }

  StateSpace::ScopedState getStartState(const Eigen::Vector3d &startPose) const
  {
    // Start state - 3 dof real vector
    StateSpace::ScopedState startState = stateSpace->createState();
    auto subState = startState.getSubStateHandle<RealVectorStateSpace>(0);
    subState.setValue(startPose);

    return startState;
  }

  StateSpace::ScopedState getGoalState(const Eigen::Vector3d &goalPose) const
  {
    // Goal state
    StateSpace::ScopedState goalState = stateSpace->createState();
    auto subState = goalState.getSubStateHandle<RealVectorStateSpace>(0);
    subState.setValue(goalPose);

    return goalState;
  }

  void setStateValue(const Eigen::Vector3d &value,
                     aikido::statespace::StateSpace::State *state) const
  {
    auto subState =
        stateSpace->getSubStateHandle<RealVectorStateSpace>(state, 0);
    subState.setValue(value);
  }

  Eigen::Vector3d getStateValue(
      aikido::statespace::StateSpace::State *state) const
  {
    auto subState =
        stateSpace->getSubStateHandle<RealVectorStateSpace>(state, 0);
    return subState.getValue();
  }

  aikido::statespace::MetaSkeletonStateSpacePtr stateSpace;
  aikido::statespace::InterpolatorPtr interpolator;
  aikido::distance::DistanceMetricPtr dmetric;
  aikido::constraint::SampleableConstraintPtr sampler;
  aikido::constraint::ProjectablePtr boundsProjection;
  aikido::constraint::TestableConstraintPtr boundsConstraint;
  aikido::constraint::TestableConstraintPtr collConstraint;
  std::shared_ptr<TranslationalProjectionConstraint> trajConstraint;
};

TEST_F(OMPLConstrainedTest, Plan)
{
  Eigen::Vector3d startPose(-3, -5, 0);
  Eigen::Vector3d goalPose(-3, 5, 0);

  StateSpace::ScopedState startState = getStartState(startPose);
  stateSpace->setStateOnMetaSkeleton(startState);
  StateSpace::ScopedState goalState = getGoalState(goalPose);

  // Plan
  auto traj = aikido::ompl::planOMPLConstrained<aikido::ompl::CRRT>(
      startState, goalState, stateSpace, interpolator,
      std::move(collConstraint), std::move(boundsConstraint),
      std::move(dmetric), std::move(sampler), std::move(boundsProjection),
      trajConstraint, 5.0, 0.1);

  // Check the first waypoint
  auto s0 = stateSpace->createState();
  traj->evaluate(0, s0);
  auto r0 = s0.getSubStateHandle<RealVectorStateSpace>(0);
  EXPECT_TRUE(r0.getValue().isApprox(startPose));

  // Check the last waypoint
  traj->evaluate(traj->getDuration(), s0);
  r0 = s0.getSubStateHandle<RealVectorStateSpace>(0);
  EXPECT_TRUE(r0.getValue().isApprox(goalPose));

  // Now check that all points along the path satisfy the constraint
//  aikido::util::StepSequence seq(0.05, true, 0, traj->getDuration());
//  for (double t : seq) {
  for(double t = 0; t < traj->getDuration(); t+= 0.05){
    traj->evaluate(t, s0);
    EXPECT_TRUE(trajConstraint->isSatisfied(s0));
  }
}
