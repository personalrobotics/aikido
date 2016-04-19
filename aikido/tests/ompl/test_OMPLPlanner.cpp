#include "OMPLTestHelpers.hpp"
#include <gtest/gtest.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/constraint/CollisionConstraint.hpp>
#include <aikido/constraint/dart.hpp>
#include <aikido/distance/DistanceMetricDefaults.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>
#include <aikido/ompl/OMPLPlanner.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <dart/dart.h>
#include <tuple>

using std::shared_ptr;
using std::make_shared;
using aikido::statespace::GeodesicInterpolator;
using RealVectorStateSpace = aikido::statespace::RealVectorStateSpace;
using StateSpace = aikido::statespace::MetaSkeletonStateSpace;
using AIKIDOStateSpace = aikido::ompl::AIKIDOGeometricStateSpace;

// Test planning for a translational robot in a world with a 1m x 1m x 1m
// box obstacle at origin
class OMPLPlannerTest : public ::testing::Test
{
public:
  virtual void SetUp()
  {
    auto robot = createTranslationalRobot();

    stateSpace = make_shared<StateSpace>(robot);
    interpolator = std::make_shared<GeodesicInterpolator>(stateSpace);

    // Collision constraint
    collConstraint = std::make_shared<MockTranslationalRobotConstraint>(
        stateSpace, Eigen::Vector3d(-0.5, -0.5, -0.5),
        Eigen::Vector3d(0.5, 0.5, 0.5));

    // Distance metric
    dmetric = aikido::distance::createDistanceMetricFor(stateSpace);

    // Sampler
    sampler =
        aikido::constraint::createSampleableBounds(stateSpace, make_rng());

    // Projectable constraint
    boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);

    // Joint limits
    boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);
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

  aikido::statespace::MetaSkeletonStateSpacePtr stateSpace;
  aikido::statespace::InterpolatorPtr interpolator;
  aikido::distance::DistanceMetricPtr dmetric;
  aikido::constraint::SampleableConstraintPtr sampler;
  aikido::constraint::ProjectablePtr boundsProjection;
  aikido::constraint::TestableConstraintPtr boundsConstraint;
  aikido::constraint::TestableConstraintPtr collConstraint;
};

class AIKIDOGeometricStateSpaceTest : public OMPLPlannerTest
{
public:
  virtual void SetUp()
  {
    OMPLPlannerTest::SetUp();
    gSpace = make_shared<AIKIDOStateSpace>(stateSpace, interpolator, dmetric,
                                           sampler, boundsConstraint,
                                           boundsProjection);
  }
  std::shared_ptr<AIKIDOStateSpace> gSpace;
};

TEST_F(OMPLPlannerTest, SimplePlan)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  StateSpace::ScopedState startState = getStartState(startPose);
  stateSpace->setStateOnMetaSkeleton(startState);

  StateSpace::ScopedState goalState = getGoalState(goalPose);

  // Plan
  auto traj = aikido::ompl::planOMPL<ompl::geometric::RRTConnect>(
      startState, goalState, stateSpace, interpolator,
      std::move(collConstraint), std::move(boundsConstraint),
      std::move(dmetric), std::move(sampler), std::move(boundsProjection), 5.0,
      0.1);

  // Check the first waypoint
  auto s0 = stateSpace->createState();
  traj->evaluate(0, s0);
  auto r0 = s0.getSubStateHandle<RealVectorStateSpace>(0);
  EXPECT_TRUE(r0.getValue().isApprox(startPose));

  // Check the last waypoint
  traj->evaluate(traj->getDuration(), s0);
  r0 = s0.getSubStateHandle<RealVectorStateSpace>(0);
  EXPECT_TRUE(r0.getValue().isApprox(goalPose));
}

TEST_F(AIKIDOGeometricStateSpaceTest, Dimension)
{
  EXPECT_EQ(3, gSpace->getDimension());
}

// TODO: Maximum Extent

// TODO: Measure

TEST_F(AIKIDOGeometricStateSpaceTest, EnforceBounds)
{
  auto state = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  Eigen::Vector3d badValue(-6, 16, 10);
  setStateValue(badValue, state->mState);

  gSpace->enforceBounds(state);
  EXPECT_TRUE(getStateValue(state->mState).isApprox(Eigen::Vector3d(-5, 5, 0)));

  Eigen::Vector3d goodValue(2, -3, 0);
  setStateValue(goodValue, state->mState);
  gSpace->enforceBounds(state);
  EXPECT_TRUE(getStateValue(state->mState).isApprox(goodValue));

  gSpace->freeState(state);
}

TEST_F(AIKIDOGeometricStateSpaceTest, SatisfiesBounds)
{
  auto state = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  Eigen::Vector3d badValue(-6, 16, 10);
  setStateValue(badValue, state->mState);
  EXPECT_FALSE(gSpace->satisfiesBounds(state));

  Eigen::Vector3d goodValue(2, -3, 0);
  setStateValue(goodValue, state->mState);
  EXPECT_TRUE(gSpace->satisfiesBounds(state));

  gSpace->freeState(state);
}

TEST_F(AIKIDOGeometricStateSpaceTest, CopyState)
{
  auto state = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  Eigen::Vector3d value(-2, 3, 0);
  setStateValue(value, state->mState);

  auto copyState = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  gSpace->copyState(copyState, state);
  EXPECT_TRUE(
      getStateValue(copyState->mState).isApprox(getStateValue(state->mState)));

  gSpace->freeState(state);
  gSpace->freeState(copyState);
}

TEST_F(AIKIDOGeometricStateSpaceTest, Distance)
{
  auto s1 = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  Eigen::Vector3d v1(-2, 3, 0);
  setStateValue(v1, s1->mState);

  auto s2 = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  Eigen::Vector3d v2(3, 4, 0);
  setStateValue(v2, s2->mState);

  EXPECT_DOUBLE_EQ((v1 - v2).norm(), gSpace->distance(s1, s2));

  gSpace->freeState(s1);
  gSpace->freeState(s2);
}

TEST_F(AIKIDOGeometricStateSpaceTest, EqualStates)
{
  auto s1 = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  Eigen::Vector3d v1(-2, 3, 0);
  setStateValue(v1, s1->mState);

  auto s2 = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  Eigen::Vector3d v2(3, 4, 0);
  setStateValue(v2, s2->mState);

  auto s3 = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  setStateValue(v1, s3->mState);

  EXPECT_TRUE(gSpace->equalStates(s1, s3));
  EXPECT_FALSE(gSpace->equalStates(s1, s2));

  gSpace->freeState(s1);
  gSpace->freeState(s2);
  gSpace->freeState(s3);
}

TEST_F(AIKIDOGeometricStateSpaceTest, Interpolate)
{
  auto s1 = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  Eigen::Vector3d v1(-2, 3, 0);
  setStateValue(v1, s1->mState);

  auto s2 = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  Eigen::Vector3d v2(3, 4, 0);
  setStateValue(v2, s2->mState);

  auto s3 = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();

  gSpace->interpolate(s1, s2, 0, s3);
  EXPECT_TRUE(getStateValue(s3->mState).isApprox(getStateValue(s1->mState)));

  gSpace->interpolate(s1, s2, 1, s3);
  EXPECT_TRUE(getStateValue(s3->mState).isApprox(getStateValue(s2->mState)));

  gSpace->interpolate(s1, s2, 0.5, s3);
  EXPECT_TRUE(getStateValue(s3->mState).isApprox(Eigen::Vector3d(0.5, 3.5, 0)));

  gSpace->freeState(s1);
  gSpace->freeState(s2);
  gSpace->freeState(s3);
}

TEST_F(AIKIDOGeometricStateSpaceTest, AllocStateSampler)
{
  ompl::base::StateSamplerPtr ssampler = gSpace->allocDefaultStateSampler();
  auto s1 = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  auto s2 = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();

  // Ensure we get two different states if we sample twice
  ssampler->sampleUniform(s1);
  ssampler->sampleUniform(s2);
  EXPECT_FALSE(getStateValue(s1->mState).isApprox(getStateValue(s2->mState)));

  EXPECT_THROW(ssampler->sampleUniformNear(s1, s2, 0.05), std::runtime_error);
  EXPECT_THROW(ssampler->sampleGaussian(s1, s2, 0.05), std::runtime_error);

  gSpace->freeState(s1);
  gSpace->freeState(s2);
}

TEST_F(AIKIDOGeometricStateSpaceTest, CopyAlloc)
{
  auto s1 = gSpace->allocState()->as<AIKIDOStateSpace::StateType>();
  Eigen::Vector3d value(-2, 3, 0);
  setStateValue(value, s1->mState);

  auto s2 = gSpace->allocState(s1->mState)->as<AIKIDOStateSpace::StateType>();
  EXPECT_TRUE(getStateValue(s1->mState).isApprox(getStateValue(s2->mState)));

  gSpace->freeState(s1);
  gSpace->freeState(s2);
}
