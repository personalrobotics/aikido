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

TEST_F(SimplifierTest, StartAndEndStatesUnchanged)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  auto startState = stateSpace->createState();
  auto subState1 = stateSpace->getSubStateHandle<R3>(startState, 0);
  subState1.setValue(startPose);

  auto goalState = stateSpace->createState();
  auto subState2 = stateSpace->getSubStateHandle<R3>(goalState, 0);
  subState2.setValue(goalPose);

  // Plan
  auto traj = aikido::planner::ompl::planOMPL<ompl::geometric::RRTConnect>(
      startState, goalState, stateSpace, interpolator, std::move(dmetric),
      std::move(sampler), std::move(collConstraint),
      std::move(boundsConstraint), std::move(boundsProjection), 5.0, 0.1);

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