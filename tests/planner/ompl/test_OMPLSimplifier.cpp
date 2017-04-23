#include "OMPLTestHelpers.hpp"
#include "../../constraint/MockConstraints.hpp"
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/planner/ompl/CRRT.hpp>
#include <aikido/planner/ompl/CRRTConnect.hpp>
#include <aikido/trajectory/Interpolated.hpp>
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

  template <typename T>
  double computeTrajLength(T& traj,
      aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
      aikido::distance::DistanceMetricPtr& dmetric)
  {
    auto stateCurrent = stateSpace->createState();
    auto stateNext = stateSpace->createState();

    double trajDistance = 0.0;
    
    for(size_t i = 0; i < traj->getDuration(); ++i)
    {
      traj->evaluate(i, stateCurrent);
      auto rCurrent = stateCurrent.getSubStateHandle<R3>(0);
      auto currentState = rCurrent.getValue();
      
      traj->evaluate(i+1, stateNext);
      auto rNext = stateNext.getSubStateHandle<R3>(0);
      auto nextState = rNext.getValue();

      trajDistance += dmetric->distance(stateCurrent, stateNext);
    }

    return trajDistance;
  }

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

} // nampespace


// Test that the start and goal positions do not change
TEST_F(SimplifierTest, EndPointsRemainUnchanged)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Original Trajectory
  auto traj = planTrajForTest(stateSpace, interpolator, dmetric, sampler, collConstraint, 
      boundsConstraint, boundsProjection, startPose, goalPose);

  // Setup for simplification
  aikido::distance::DistanceMetricPtr dmetric = aikido::distance::createDistanceMetric(stateSpace);
  aikido::constraint::SampleablePtr sampler = aikido::constraint::createSampleableBounds(stateSpace, make_rng()); 
  aikido::constraint::ProjectablePtr boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);
  aikido::constraint::TestablePtr boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);
  aikido::constraint::TestablePtr collConstraint = std::make_shared<MockTranslationalRobotConstraint>(
        stateSpace, Eigen::Vector3d(-0.1, -0.1, -0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1));

  auto simplifiedPair = aikido::planner::ompl::simplifyOMPL(
      stateSpace, interpolator, std::move(dmetric), std::move(sampler), 
      std::move(collConstraint), std::move(boundsConstraint), std::move(boundsProjection),
      0.1, 5.0, 10, traj);

  // Simplification results
  auto simplifiedTraj = std::move(simplifiedPair.first);
  bool shorten_success = simplifiedPair.second;

  // Check the first waypoint
  auto s0 = stateSpace->createState();
  simplifiedTraj->evaluate(0, s0);
  auto r0 = s0.getSubStateHandle<R3>(0);
  EXPECT_TRUE(r0.getValue().isApprox(startPose));

  // Check the last waypoint
  simplifiedTraj->evaluate(simplifiedTraj->getDuration(), s0);
  r0 = s0.getSubStateHandle<R3>(0);
  EXPECT_TRUE(r0.getValue().isApprox(goalPose));

}


// Test that the boolean returned is in agreement with shortening process
TEST_F(SimplifierTest, ShortenThreeWayPointTraj)
{

  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  auto traj = planTrajForTest(stateSpace, interpolator, dmetric, sampler, collConstraint, 
      boundsConstraint, boundsProjection, startPose, goalPose);

  // Setup for simplification
  aikido::distance::DistanceMetricPtr dmetric = aikido::distance::createDistanceMetric(stateSpace);
  aikido::constraint::SampleablePtr sampler = aikido::constraint::createSampleableBounds(stateSpace, make_rng()); 
  aikido::constraint::ProjectablePtr boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);
  aikido::constraint::TestablePtr boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);
  aikido::constraint::TestablePtr collConstraint = std::make_shared<MockTranslationalRobotConstraint>(
        stateSpace, Eigen::Vector3d(-0.1, -0.1, -0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1));

  auto simplifiedPair = aikido::planner::ompl::simplifyOMPL(
      stateSpace, interpolator, std::move(dmetric), std::move(sampler), 
      std::move(collConstraint), std::move(boundsConstraint), std::move(boundsProjection),
      0.1, 5.0, 10, traj);

  // Simplification results
  auto simplifiedTraj = std::move(simplifiedPair.first);
  bool shorten_success = simplifiedPair.second;

  aikido::distance::DistanceMetricPtr Ddmetric = aikido::distance::createDistanceMetric(stateSpace);
  double trajDistance = computeTrajLength<aikido::trajectory::InterpolatedPtr>(traj, stateSpace, Ddmetric);
  double simplifiedTrajDistance = computeTrajLength<std::unique_ptr<aikido::trajectory::Interpolated>>(simplifiedTraj, stateSpace, Ddmetric);

  EXPECT_TRUE(shorten_success == (simplifiedTrajDistance < trajDistance));  
}


// Test that a path with two waypoints (straight line) does not get shortened
TEST_F(SimplifierTest, ShortenTwoWayPointTraj)
{

  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  auto startState = stateSpace->createState();
  auto subState1 = stateSpace->getSubStateHandle<R3>(startState, 0);
  subState1.setValue(startPose);

  auto goalState = stateSpace->createState();
  auto subState2 = stateSpace->getSubStateHandle<R3>(goalState, 0);
  subState2.setValue(goalPose);

  auto traj = std::make_shared<aikido::trajectory::Interpolated>(
        stateSpace, interpolator);

  traj->addWaypoint(0, startState);
  traj->addWaypoint(1, goalState);

  // TODO: unique pointers are lost. Is there a better way to go about this?
  aikido::distance::DistanceMetricPtr _dmetric = aikido::distance::createDistanceMetric(stateSpace);
  aikido::constraint::SampleablePtr _sampler = aikido::constraint::createSampleableBounds(stateSpace, make_rng()); 
  aikido::constraint::ProjectablePtr _boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);
  aikido::constraint::TestablePtr _boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);
  aikido::constraint::TestablePtr _collConstraint = std::make_shared<MockTranslationalRobotConstraint>(
        stateSpace, Eigen::Vector3d(-0.1, -0.1, -0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1));  

  auto simplifiedPair = aikido::planner::ompl::simplifyOMPL(
      stateSpace, interpolator, std::move(_dmetric), std::move(_sampler), 
      std::move(_collConstraint), std::move(_boundsConstraint), std::move(_boundsProjection),
      0.1, 5.0, 10, traj);
  auto simplifiedTraj = std::move(simplifiedPair.first);
  bool shorten_success = simplifiedPair.second;
  
  EXPECT_TRUE(!shorten_success);  
}