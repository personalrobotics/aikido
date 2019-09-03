#include <aikido/common/StepSequence.hpp>
#include <aikido/planner/ompl/MotionValidator.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include "../../constraint/MockConstraints.hpp"
#include "OMPLTestHelpers.hpp"
#include "eigen_tests.hpp"

static const double eigenTolerance{1e-6};

using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::planner::ompl::getSpaceInformation;

namespace {

aikido::trajectory::InterpolatedPtr constructTrajectory(
    aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr _stateSpace,
    aikido::statespace::InterpolatorPtr _interpolator,
    const Eigen::Vector3d& _startPose,
    const Eigen::Vector3d& _midwayPose,
    const Eigen::Vector3d& _goalPose)
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
} // namespace

// Test if the durations of the two trajectories remains constant
TEST_F(PlannerTest, OMPLNumWaypointsUnchanged)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d midwayPose(0, -2, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Construct a test trajectory
  auto traj = constructTrajectory(
      stateSpace, interpolator, startPose, midwayPose, goalPose);

  // Get the ompl state space
  auto si = getSpaceInformation(
      stateSpace,
      interpolator,
      std::move(dmetric),
      std::move(sampler),
      std::move(collConstraint),
      std::move(boundsConstraint),
      std::move(boundsProjection),
      0.1);

  auto omplTraj = aikido::planner::ompl::toOMPLTrajectory(traj, si);

  EXPECT_TRUE(omplTraj.getStateCount() == traj->getNumWaypoints());
}

// Tests that toOMPLTrajectory function does not alter the waypoints
TEST_F(PlannerTest, OMPLStatesRemainUnchaged)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d midwayPose(0, -2, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Construct a test trajectory
  auto traj = constructTrajectory(
      stateSpace, interpolator, startPose, midwayPose, goalPose);

  // Get the ompl state space
  auto si = getSpaceInformation(
      stateSpace,
      interpolator,
      std::move(dmetric),
      std::move(sampler),
      std::move(collConstraint),
      std::move(boundsConstraint),
      std::move(boundsProjection),
      0.1);

  auto omplTraj = aikido::planner::ompl::toOMPLTrajectory(traj, si);

  // Ensure equal waypoints
  ASSERT_TRUE(traj->getNumWaypoints() == omplTraj.getStateCount());

  // Match every point
  for (std::size_t idx = 0; idx < traj->getNumWaypoints(); ++idx)
  {
    auto s0
        = static_cast<const CartesianProduct::State*>(traj->getWaypoint(idx));
    auto r0 = stateSpace->getSubStateHandle<R3>(s0, 0);
    auto stateInterpolated = r0.getValue();

    auto state = omplTraj.getState(idx);
    auto stateOMPL = getTranslationalState(stateSpace, state);

    EXPECT_EIGEN_EQUAL(stateInterpolated, stateOMPL, eigenTolerance);
  }
}

// Tests that toInterpolatedTrajectory function does not alter the duration
TEST_F(PlannerTest, InterpolatedNumWaypointsUnchaged)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d midwayPose(0, -2, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Construct a test trajectory
  auto traj = constructTrajectory(
      stateSpace, interpolator, startPose, midwayPose, goalPose);

  // Get the ompl state space
  auto si = getSpaceInformation(
      stateSpace,
      interpolator,
      std::move(dmetric),
      std::move(sampler),
      std::move(collConstraint),
      std::move(boundsConstraint),
      std::move(boundsProjection),
      0.1);

  auto omplTraj = aikido::planner::ompl::toOMPLTrajectory(traj, si);

  auto interpolatedTraj
      = aikido::planner::ompl::toInterpolatedTrajectory(omplTraj, interpolator);

  EXPECT_TRUE(omplTraj.getStateCount() == interpolatedTraj->getNumWaypoints());
}

// Tests that toInterpolatedTrajectory function does not alter the waypoints
TEST_F(PlannerTest, InterpolatedStatesRemainUnchaged)
{
  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d midwayPose(0, -2, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  // Construct a test trajectory
  auto traj = constructTrajectory(
      stateSpace, interpolator, startPose, midwayPose, goalPose);

  // Get the ompl state space
  auto si = getSpaceInformation(
      stateSpace,
      interpolator,
      std::move(dmetric),
      std::move(sampler),
      std::move(collConstraint),
      std::move(boundsConstraint),
      std::move(boundsProjection),
      0.1);

  auto omplTraj = aikido::planner::ompl::toOMPLTrajectory(traj, si);

  auto interpolatedTraj
      = aikido::planner::ompl::toInterpolatedTrajectory(omplTraj, interpolator);

  // Ensure equal waypoints
  ASSERT_TRUE(interpolatedTraj->getNumWaypoints() == omplTraj.getStateCount());

  // Match every point
  for (std::size_t idx = 0; idx < interpolatedTraj->getNumWaypoints(); ++idx)
  {
    auto s0 = static_cast<const CartesianProduct::State*>(
        interpolatedTraj->getWaypoint(idx));
    auto r0 = stateSpace->getSubStateHandle<R3>(s0, 0);
    auto stateInterpolated = r0.getValue();

    auto state = omplTraj.getState(idx);
    auto stateOMPL = getTranslationalState(stateSpace, state);

    EXPECT_EIGEN_EQUAL(stateInterpolated, stateOMPL, eigenTolerance);
  }
}
