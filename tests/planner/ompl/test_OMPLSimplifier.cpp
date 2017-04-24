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
#include <dart/dart.hpp>
#include "eigen_tests.hpp"

static const double eigenTolerance{1e-6};

using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::planner::ompl::getSpaceInformation;
using aikido::planner::ompl::CRRT;
using aikido::planner::ompl::CRRTConnect;

namespace {

  template <typename T>
  double computeTrajLength(const T& traj,
      aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
      aikido::distance::DistanceMetricPtr& dmetric)
  {
    auto stateCurrent = stateSpace->createState();
    auto stateNext = stateSpace->createState();

    double trajDistance = 0.0;
    
    if(traj->getNumWaypoints())
    {
      for(size_t i = 0; i < traj->getNumWaypoints() - 1; ++i)
      {
        traj->evaluate(i, stateCurrent);   
        traj->evaluate(i+1, stateNext);
        trajDistance += dmetric->distance(stateCurrent, stateNext);
      }  
    }
    

    return trajDistance;
  }

  aikido::trajectory::InterpolatedPtr constructTrajectory(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
      aikido::statespace::InterpolatorPtr _interpolator,
      const Eigen::Vector3d& _startPose,
      const Eigen::Vector3d& _midwayPose, 
      const Eigen::Vector3d& _goalPose          
    )
  {

    auto startState = _stateSpace->createState();
    auto subState1 = _stateSpace->getSubStateHandle<R3>(startState, 0);
    subState1.setValue(_startPose);

    auto midwayState = _stateSpace->createState();
    auto subState2 = _stateSpace->getSubStateHandle<R3>(midwayState, 0);
    subState2.setValue(_midwayPose);

    auto goalState = _stateSpace->createState();
    auto subState3 = _stateSpace->getSubStateHandle<R3>(goalState, 0);
    subState3.setValue(_goalPose);

    // Construct Trajectory
    auto returnInterpolated = std::make_shared<aikido::trajectory::Interpolated>(
        std::move(_stateSpace), std::move(_interpolator));

    returnInterpolated->addWaypoint(0, startState);
    returnInterpolated->addWaypoint(1, midwayState);
    returnInterpolated->addWaypoint(2, goalState);

    return returnInterpolated;
  }

} // nampespace


// Test that the start and goal positions do not change
TEST_F(SimplifierTest, EndPointsRemainUnchanged)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d midwayPose(0, -2, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Construct a test trajectory
  auto traj = constructTrajectory(stateSpace, interpolator, startPose, midwayPose, goalPose);

  // Simplify the trajectory
  auto simplifiedPair = aikido::planner::ompl::simplifyOMPL(
      stateSpace, interpolator, std::move(dmetric), std::move(sampler), 
      std::move(collConstraint), std::move(boundsConstraint), std::move(boundsProjection),
      0.1, 5.0, 10, traj);

  // Simplification results
  auto simplifiedTraj = std::move(simplifiedPair.first);

  // Check the first waypoint
  auto s0 = stateSpace->createState();
  simplifiedTraj->evaluate(0, s0);
  auto r0 = s0.getSubStateHandle<R3>(0);
  EXPECT_EIGEN_EQUAL(r0.getValue(), startPose, eigenTolerance);

  // Check the last waypoint
  simplifiedTraj->evaluate(simplifiedTraj->getDuration(), s0);
  r0 = s0.getSubStateHandle<R3>(0);
  EXPECT_EIGEN_EQUAL(r0.getValue(), goalPose, eigenTolerance);

}


// Test that the boolean returned is in agreement with shortening process
TEST_F(SimplifierTest, ShortenThreeWayPointTraj)
{

  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d midwayPose(0, -2, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Construct a test trajectory
  auto traj = constructTrajectory(stateSpace, interpolator, startPose, midwayPose, goalPose);

  // Simplify the trajectory 
  auto simplifiedPair = aikido::planner::ompl::simplifyOMPL(
      stateSpace, interpolator, std::move(dmetric), std::move(sampler), 
      std::move(collConstraint), std::move(boundsConstraint), std::move(boundsProjection),
      0.1, 20.0, 20, traj);

  // Simplification results
  auto simplifiedTraj = std::move(simplifiedPair.first);
  bool shorten_success = simplifiedPair.second;

  aikido::distance::DistanceMetricPtr dmetric = aikido::distance::createDistanceMetric(stateSpace);
  double trajDistance = computeTrajLength<aikido::trajectory::InterpolatedPtr>(traj, stateSpace, dmetric);
  double simplifiedTrajDistance = computeTrajLength<std::unique_ptr<aikido::trajectory::Interpolated>>(simplifiedTraj, stateSpace, dmetric);

  EXPECT_TRUE(shorten_success);  
}


// Test that a path with two waypoints (straight line) does not get shortened
TEST_F(SimplifierTest, ShortenTwoWayPointTraj)
{

  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d midwayPose(0, 0, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Construct a test trajectory
  auto traj = constructTrajectory(stateSpace, interpolator, startPose, midwayPose, goalPose);

  // Simplify the trajectory 
  auto simplifiedPair = aikido::planner::ompl::simplifyOMPL(
      stateSpace, interpolator, std::move(dmetric), std::move(sampler), 
      std::move(collConstraint), std::move(boundsConstraint), std::move(boundsProjection),
      0.1, 20.0, 20, traj);

  bool shorten_success = simplifiedPair.second;
  EXPECT_TRUE(!shorten_success);  
}