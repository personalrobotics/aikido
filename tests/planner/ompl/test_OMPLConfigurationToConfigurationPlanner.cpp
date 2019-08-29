#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <aikido/common/StepSequence.hpp>
#include <aikido/constraint.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/ompl/MotionValidator.hpp>
#include <aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include "../../constraint/MockConstraints.hpp"
#include "OMPLTestHelpers.hpp"

using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::planner::ConfigurationToConfiguration;
using aikido::planner::ompl::getSpaceInformation;
using aikido::planner::ompl::ompl_dynamic_pointer_cast;
using aikido::planner::ompl::OMPLConfigurationToConfigurationPlanner;

//==============================================================================
class UnknownProblem : public aikido::planner::Problem
{
public:
  UnknownProblem()
    : aikido::planner::Problem(std::make_shared<aikido::statespace::SO2>())
  {
    // Do nothing
  }

  const std::string& getType() const override
  {
    return getStaticType();
  }

  static const std::string& getStaticType()
  {
    static std::string name("UnknownProblem");
    return name;
  }
};

//==============================================================================
TEST_F(PlannerTest, CanSolveProblems)
{
  // Tests if the planner can solve the toConfiguration problem.

  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  auto startState = stateSpace->createState();
  auto subState1 = stateSpace->getSubStateHandle<R3>(startState, 0);
  subState1.setValue(startPose);

  auto goalState = stateSpace->createState();
  auto subState2 = stateSpace->getSubStateHandle<R3>(goalState, 0);
  subState2.setValue(goalPose);

  auto planner = std::make_shared<
      OMPLConfigurationToConfigurationPlanner<ompl::geometric::RRTConnect>>(
      stateSpace,
      nullptr,
      interpolator,
      std::move(dmetric),
      std::move(sampler),
      std::move(boundsConstraint),
      std::move(boundsProjection),
      0.1);

  auto problem = ConfigurationToConfiguration(
      stateSpace, startState, goalState, collConstraint);
  auto unknownProblem = UnknownProblem();

  EXPECT_TRUE(planner->canSolve(problem));
  EXPECT_FALSE(planner->canSolve(unknownProblem));
}

//==============================================================================
TEST_F(PlannerTest, PlanToConfiguration)
{
  // Tests correctness of the planning procedure.

  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  auto startState = stateSpace->createState();
  auto subState1 = stateSpace->getSubStateHandle<R3>(startState, 0);
  subState1.setValue(startPose);

  auto goalState = stateSpace->createState();
  auto subState2 = stateSpace->getSubStateHandle<R3>(goalState, 0);
  subState2.setValue(goalPose);

  auto problem = ConfigurationToConfiguration(
      stateSpace, startState, goalState, collConstraint);
  auto planner = std::make_shared<
      OMPLConfigurationToConfigurationPlanner<ompl::geometric::RRTConnect>>(
      stateSpace,
      nullptr,
      interpolator,
      std::move(dmetric),
      std::move(sampler),
      std::move(boundsConstraint),
      std::move(boundsProjection),
      0.1);
  auto traj = planner->plan(problem);

  // Check the first waypoint
  auto s0 = stateSpace->createState();
  traj->evaluate(0, s0);
  auto r0 = s0.getSubStateHandle<R3>(0);
  EXPECT_TRUE(r0.getValue().isApprox(startPose));

  // Check the last waypoint
  traj->evaluate(traj->getDuration(), s0);
  r0 = s0.getSubStateHandle<R3>(0);
  EXPECT_TRUE(r0.getValue().isApprox(goalPose));
}

//==============================================================================
TEST_F(PlannerTest, PlannerSpecificParameters)
{
  // Tests if replanning with the same planner is done correctly.
  // Tests if planner specific parameters are appropriately set.

  Eigen::Vector3d startPose(-5, -5, 0);
  Eigen::Vector3d goalPose(5, 5, 0);

  auto startState = stateSpace->createState();
  auto subState1 = stateSpace->getSubStateHandle<R3>(startState, 0);
  subState1.setValue(startPose);

  auto goalState = stateSpace->createState();
  auto subState2 = stateSpace->getSubStateHandle<R3>(goalState, 0);
  subState2.setValue(goalPose);

  auto freeConstraint = std::make_shared<PassingConstraint>(stateSpace);

  auto problem = ConfigurationToConfiguration(
      stateSpace, startState, goalState, freeConstraint);

  auto planner = std::make_shared<
      OMPLConfigurationToConfigurationPlanner<ompl::geometric::RRTConnect>>(
      stateSpace,
      nullptr,
      interpolator,
      std::move(dmetric),
      std::move(sampler),
      std::move(boundsConstraint),
      std::move(boundsProjection),
      0.1);
  auto rrtPlanner
      = planner->getOMPLPlanner()->as<ompl::geometric::RRTConnect>();
  if (rrtPlanner)
    rrtPlanner->setRange(0.5);

  auto traj = planner->plan(problem);

  // Check the first waypoint
  auto s0 = stateSpace->createState();
  traj->evaluate(0, s0);
  auto r0 = s0.getSubStateHandle<R3>(0);
  EXPECT_TRUE(r0.getValue().isApprox(startPose));

  // Check the last waypoint
  traj->evaluate(traj->getDuration(), s0);
  r0 = s0.getSubStateHandle<R3>(0);
  EXPECT_TRUE(r0.getValue().isApprox(goalPose));

  // Convert the trajectory to Interpolated and check numWaypoints
  auto interpolated
      = dynamic_cast<aikido::trajectory::Interpolated*>(traj.get());
  if (interpolated)
  {
    EXPECT_TRUE(rrtPlanner->getRange() == 0.5);
    EXPECT_TRUE(interpolated->getNumWaypoints() > 3);
  }

  rrtPlanner->setRange(std::numeric_limits<double>::infinity());
  auto newTraj = planner->plan(problem);

  // Convert the trajectory to Interpolated and check numWaypoints
  auto newInterpolated
      = dynamic_cast<aikido::trajectory::Interpolated*>(newTraj.get());
  if (newInterpolated)
  {
    EXPECT_TRUE(
        rrtPlanner->getRange() == std::numeric_limits<double>::infinity());
    EXPECT_TRUE(newInterpolated->getNumWaypoints() == 3);
  }
}
