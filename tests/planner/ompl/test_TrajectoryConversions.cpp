#include "OMPLTestHelpers.hpp"
#include "../../constraint/MockConstraints.hpp"
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/planner/ompl/CRRT.hpp>
#include <aikido/planner/ompl/CRRTConnect.hpp>
#include <aikido/constraint/uniform/RnBoxConstraint.hpp>
#include <aikido/constraint/CartesianProductSampleable.hpp>
#include <aikido/constraint/CartesianProductTestable.hpp>
#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/util/StepSequence.hpp>
#include <aikido/planner/ompl/MotionValidator.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::planner::ompl::getSpaceInformation;
using aikido::planner::ompl::CRRT;
using aikido::planner::ompl::CRRTConnect;

namespace {

  aikido::trajectory::InterpolatedPtr planTrajForTest(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
      aikido::statespace::InterpolatorPtr& _interpolator,
      aikido::distance::DistanceMetricPtr& _dmetric,
      aikido::constraint::SampleablePtr& _sampler,
      aikido::constraint::TestablePtr& _collConstraint,
      aikido::constraint::TestablePtr& _boundsConstraint,
      aikido::constraint::ProjectablePtr& _boundsProjection,
      Eigen::Vector3d _startPose, Eigen::Vector3d _goalPose          
    )
  {

    auto startState = _stateSpace->createState();
    auto subState1 = _stateSpace->getSubStateHandle<R3>(startState, 0);
    subState1.setValue(_startPose);

    auto goalState = _stateSpace->createState();
    auto subState2 = _stateSpace->getSubStateHandle<R3>(goalState, 0);
    subState2.setValue(_goalPose);

    // Plan
    auto traj = aikido::planner::ompl::planOMPL<ompl::geometric::RRTConnect>(
        startState, goalState, _stateSpace, _interpolator, std::move(_dmetric),
        std::move(_sampler), std::move(_collConstraint),
        std::move(_boundsConstraint), std::move(_boundsProjection), 5.0, 0.1);

    return traj;

  }

}

// Test if the durations of the two trajectories remains constant
TEST_F(PlannerTest, OMPLDurationRemainsUnchanged)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Original Trajectory
  auto traj = planTrajForTest(stateSpace, interpolator, dmetric, sampler, collConstraint, 
      boundsConstraint, boundsProjection, startPose, goalPose);

  // Setup
  aikido::distance::DistanceMetricPtr _dmetric = aikido::distance::createDistanceMetric(stateSpace);
  aikido::constraint::SampleablePtr _sampler = aikido::constraint::createSampleableBounds(stateSpace, make_rng()); 
  aikido::constraint::ProjectablePtr _boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);
  aikido::constraint::TestablePtr _boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);
  aikido::constraint::TestablePtr _collConstraint = std::make_shared<MockTranslationalRobotConstraint>(
        stateSpace, Eigen::Vector3d(-0.1, -0.1, -0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1));

  // Get the ompl state space
  auto si = getSpaceInformation(
      stateSpace,
      interpolator,
      std::move(_dmetric),
      std::move(_sampler),
      std::move(_collConstraint),
      std::move(_boundsConstraint),
      std::move(_boundsProjection),
      0.1);

  auto omplTraj = aikido::planner::ompl::toOMPLTrajectory(
      traj, si);

  // Note that getDuration() indexes from 0 while getStateCount() indexes from 1
  EXPECT_TRUE(omplTraj.getStateCount() == (traj->getDuration())+1);

}

// Tests that toOMPLTrajectory function does not alter the waypoints 
TEST_F(PlannerTest, OMPLStatesRemainUnchaged)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Original Trajectory
  auto traj = planTrajForTest(stateSpace, interpolator, dmetric, sampler, collConstraint, 
      boundsConstraint, boundsProjection, startPose, goalPose);

  // Setup
  aikido::distance::DistanceMetricPtr _dmetric = aikido::distance::createDistanceMetric(stateSpace);
  aikido::constraint::SampleablePtr _sampler = aikido::constraint::createSampleableBounds(stateSpace, make_rng()); 
  aikido::constraint::ProjectablePtr _boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);
  aikido::constraint::TestablePtr _boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);
  aikido::constraint::TestablePtr _collConstraint = std::make_shared<MockTranslationalRobotConstraint>(
        stateSpace, Eigen::Vector3d(-0.1, -0.1, -0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1));

  // Get the ompl state space
  auto si = getSpaceInformation(
      stateSpace,
      interpolator,
      std::move(_dmetric),
      std::move(_sampler),
      std::move(_collConstraint),
      std::move(_boundsConstraint),
      std::move(_boundsProjection),
      0.1);

  auto omplTraj = aikido::planner::ompl::toOMPLTrajectory(
      traj, si);
  
  // Match every point
  auto s0 = stateSpace->createState();
  for(size_t idx = 0; idx < traj->getDuration(); ++idx)
  {
    traj->evaluate(idx, s0);
    auto r0 = s0.getSubStateHandle<R3>(0);
    auto stateInterpolated = r0.getValue();

    auto state = omplTraj.getState(idx);
    auto stateOMPL = getTranslationalState(stateSpace, state);
    
    EXPECT_TRUE(stateInterpolated.isApprox(stateOMPL));
  }
}


// Tests that toInterpolatedTrajectory function does not alter the duration
TEST_F(PlannerTest, InterpolatedDurationRemainsUnchaged)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Original Trajectory
  auto traj = planTrajForTest(stateSpace, interpolator, dmetric, sampler, collConstraint, 
      boundsConstraint, boundsProjection, startPose, goalPose);

  // Setup
  aikido::distance::DistanceMetricPtr _dmetric = aikido::distance::createDistanceMetric(stateSpace);
  aikido::constraint::SampleablePtr _sampler = aikido::constraint::createSampleableBounds(stateSpace, make_rng()); 
  aikido::constraint::ProjectablePtr _boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);
  aikido::constraint::TestablePtr _boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);
  aikido::constraint::TestablePtr _collConstraint = std::make_shared<MockTranslationalRobotConstraint>(
        stateSpace, Eigen::Vector3d(-0.1, -0.1, -0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1));

  // Get the ompl state space
  auto si = getSpaceInformation(
      stateSpace,
      interpolator,
      std::move(_dmetric),
      std::move(_sampler),
      std::move(_collConstraint),
      std::move(_boundsConstraint),
      std::move(_boundsProjection),
      0.1);

  auto omplTraj = aikido::planner::ompl::toOMPLTrajectory(
      traj, si);

  auto interpolatedTraj = aikido::planner::ompl::toInterpolatedTrajectory(
  omplTraj, interpolator);

  EXPECT_TRUE(omplTraj.getStateCount() == (interpolatedTraj->getDuration()+1));
}


// Tests that toInterpolatedTrajectory function does not alter the waypoints
TEST_F(PlannerTest, InterpolatedStatesRemainUnchaged)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Original Trajectory
  auto traj = planTrajForTest(stateSpace, interpolator, dmetric, sampler, collConstraint, 
      boundsConstraint, boundsProjection, startPose, goalPose);

  // Setup
  aikido::distance::DistanceMetricPtr _dmetric = aikido::distance::createDistanceMetric(stateSpace);
  aikido::constraint::SampleablePtr _sampler = aikido::constraint::createSampleableBounds(stateSpace, make_rng()); 
  aikido::constraint::ProjectablePtr _boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);
  aikido::constraint::TestablePtr _boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);
  aikido::constraint::TestablePtr _collConstraint = std::make_shared<MockTranslationalRobotConstraint>(
        stateSpace, Eigen::Vector3d(-0.1, -0.1, -0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1));

  // Get the ompl state space
  auto si = getSpaceInformation(
      stateSpace,
      interpolator,
      std::move(_dmetric),
      std::move(_sampler),
      std::move(_collConstraint),
      std::move(_boundsConstraint),
      std::move(_boundsProjection),
      0.1);

  auto omplTraj = aikido::planner::ompl::toOMPLTrajectory(
      traj, si);

  auto interpolatedTraj = aikido::planner::ompl::toInterpolatedTrajectory(
  omplTraj, interpolator);

  // Match every point
  auto s0 = stateSpace->createState();
  for(size_t idx = 0; idx < interpolatedTraj->getDuration(); ++idx)
  {
    interpolatedTraj->evaluate(idx, s0);
    auto r0 = s0.getSubStateHandle<R3>(0);
    auto stateInterpolated = r0.getValue();

    auto state = omplTraj.getState(idx);
    auto stateOMPL = getTranslationalState(stateSpace, state);
    
    EXPECT_TRUE(stateInterpolated.isApprox(stateOMPL));
  }

}