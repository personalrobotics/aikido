#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <aikido/common/StepSequence.hpp>
#include <aikido/constraint.hpp>
#include <aikido/planner/ompl/MotionValidator.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp>
#include "../../constraint/MockConstraints.hpp"
#include "OMPLTestHelpers.hpp"

using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::planner::ompl::getSpaceInformation;
using aikido::planner::ompl::ompl_dynamic_pointer_cast;
using aikido::planner::ompl::OMPLConfigurationToConfigurationPlanner;
using aikido::planner::ConfigurationToConfiguration;

TEST_F(PlannerTest, PlanToConfiguration)
{
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
  auto planner = std::make_shared<OMPLConfigurationToConfigurationPlanner<ompl::geometric::RRTConnect>>(
      stateSpace, interpolator, 
      std::move(dmetric),
      std::move(sampler),
      nullptr,
      std::move(boundsConstraint),
      std::move(boundsProjection),
      0.1);
  auto traj = planner->plan(problem);
  EXPECT_TRUE(true);

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
