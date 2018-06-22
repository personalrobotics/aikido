#include <iostream>
#include <fstream>

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <dart/math/Constants.hpp>


#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/planner/kinodynamics/KinodynamicPlanner.hpp"
#include "aikido/planner/kinodynamics/dimt/DoubleIntegratorMinimumTime.h"
#include "aikido/planner/kinodynamics/ompl/DimtStateSpace.hpp"
#include "aikido/planner/kinodynamics/ompl/MyInformedRRTstar.hpp"
#include "aikido/planner/kinodynamics/ompl/MyOptimizationObjective.hpp"
#include "aikido/planner/kinodynamics/sampler/HitAndRunSampler.hpp"
#include "aikido/planner/ompl/BackwardCompatibility.hpp"
#include "aikido/planner/ompl/MotionValidator.hpp"
#include "aikido/planner/ompl/Planner.hpp"
#include "aikido/planner/ompl/StateValidityChecker.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "KinodynamicUtil.hpp"

using aikido::planner::ompl::ompl_make_shared;
using dart::dynamics::MetaSkeletonPtr;
using aikido::statespace::dart::MetaSkeletonStateSaver;

namespace aikido {
namespace planner {
namespace kinodynamics {

class StateValidityChecker : public ompl::StateValidityChecker
{
public:
  StateValidityChecker(
      const ::ompl::base::SpaceInformationPtr& si,
      constraint::TestablePtr constraint)
    : ompl::StateValidityChecker(si, constraint)
  {
  }

  bool isValid(const ::ompl::base::State* state) const override
  {
    if (!si_->getStateSpace()->satisfiesBounds(state))
    {
      return false;
    }

    // create aikido state from ompl state
    auto stateSpace = mConstraint->getStateSpace();
    auto aikidoState = stateSpace->createState();
    Eigen::VectorXd config(stateSpace->getDimension());
    for (std::size_t i = 0; i < stateSpace->getDimension(); i++)
    {
      config[i] = state->as<::ompl::base::RealVectorStateSpace::StateType>()
                      ->values[i];
    }
    stateSpace->expMap(config, aikidoState);
    return mConstraint->isSatisfied(aikidoState);
  }
};

::ompl::base::State* allocState(
    const ::ompl::base::SpaceInformationPtr si,
    const Eigen::VectorXd& stateVec,
    const Eigen::VectorXd& velocityVec)
{
  ::ompl::base::State* new_state = si->getStateSpace()->allocState();
  for (uint i = 0; i < stateVec.size(); i++)
  {
    new_state->as<::ompl::base::RealVectorStateSpace::StateType>()->values[i]
        = stateVec[i];
  }
  for (uint i = 0; i < stateVec.size(); i++)
  {
    new_state->as<::ompl::base::RealVectorStateSpace::StateType>()
        ->values[i + stateVec.size()]
        = velocityVec[i];
  }

  return new_state;
}

::ompl::base::SpaceInformationPtr getSpaceInformation(
    DIMTPtr dimt,
    MetaSkeletonPtr skeleton,
    constraint::TestablePtr validityConstraint,
    double maxDistanceBtwValidityChecks)
{
  // construct the state space we are planning in
  ::ompl::base::StateSpacePtr space
      = ompl_make_shared<::ompl::base::DimtStateSpace>(dimt);

  ::ompl::base::RealVectorBounds bounds(skeleton->getNumDofs() * 2);
  for (std::size_t i = 0; i < skeleton->getNumDofs(); i++)
  {
    bounds.setHigh(i, skeleton->getPositionUpperLimit(i));
    bounds.setLow(i, skeleton->getPositionLowerLimit(i));

    bounds.setHigh(
        skeleton->getNumDofs() + i, skeleton->getVelocityUpperLimit(i));
    bounds.setLow(
        skeleton->getNumDofs() + i, skeleton->getVelocityLowerLimit(i));
  }

  space->as<::ompl::base::DimtStateSpace>()->setBounds(bounds);
  ::ompl::base::SpaceInformationPtr si
      = ompl_make_shared<::ompl::base::SpaceInformation>(space);

  // Validity checking
  ::ompl::base::StateValidityCheckerPtr vchecker
      = ompl_make_shared<StateValidityChecker>(si, validityConstraint);
  si->setStateValidityChecker(vchecker);

  ::ompl::base::MotionValidatorPtr mvalidator
      = ompl_make_shared<aikido::planner::ompl::MotionValidator>(
          si, maxDistanceBtwValidityChecks);
  si->setMotionValidator(mvalidator);
  si->setStateValidityCheckingResolution(0.001);
  si->setup();

  return si;
}

::ompl::base::ProblemDefinitionPtr createProblem(
    ::ompl::base::SpaceInformationPtr si,
    const ::ompl::base::State* start,
    const ::ompl::base::State* goal)
{
  ::ompl::base::ScopedState<::ompl::base::RealVectorStateSpace> startState(
      si->getStateSpace(), start);
  ::ompl::base::ScopedState<::ompl::base::RealVectorStateSpace> goalState(
      si->getStateSpace(), goal);

  // Set up the final problem with the full optimization objective
  ::ompl::base::ProblemDefinitionPtr pdef
      = ompl_make_shared<::ompl::base::ProblemDefinition>(si);
  pdef->setStartAndGoalStates(startState, goalState);

  return pdef;
}

const ::ompl::base::OptimizationObjectivePtr createDimtOptimizationObjective(
    ::ompl::base::SpaceInformationPtr si,
    DIMTPtr dimt,
    const ::ompl::base::State* start,
    const ::ompl::base::State* goal)
{
  const ::ompl::base::OptimizationObjectivePtr base_opt
      = ompl_make_shared<::ompl::base::DimtObjective>(si, start, goal, dimt);
  return base_opt;
}

std::unique_ptr<aikido::trajectory::Spline> concatenateTwoPaths(
    const ::ompl::base::PathPtr& path1,
    const ::ompl::base::PathPtr& path2,
    const DIMTPtr& dimt,
    const statespace::dart::MetaSkeletonStateSpacePtr& metaSkeletonStateSpace,
    double interpolateStepSize = 0.05)
{
  if (path1 == nullptr || path2 == nullptr)
  {
    return nullptr;
  }

  // converting path 1 
  std::vector<Eigen::VectorXd> points;
  std::vector<double> times;

  std::vector<Eigen::VectorXd> path1points;
  std::vector<double> path1times;
  ::ompl::geometric::PathGeometric* geopath1
      = path1->as<::ompl::geometric::PathGeometric>();
  convertPathToSequentialStates(geopath1, dimt, interpolateStepSize,
                                path1points, path1times);
 
  // converting path 2
  std::vector<Eigen::VectorXd> path2points;
  std::vector<double> path2times;
  ::ompl::geometric::PathGeometric* geopath2
      = path2->as<::ompl::geometric::PathGeometric>();
  convertPathToSequentialStates(geopath2, dimt, interpolateStepSize,
                                path2points, path2times);
  
  // merge two sets
  std::transform(
      path2times.begin(),
      path2times.end(),
      path2times.begin(),
      std::bind2nd(std::plus<double>(), path1times.back()));
  
  points.insert(points.end(), path1points.begin(), path1points.end());
  times.insert(times.end(), path1times.begin(), path1times.end());
  points.insert(points.end(), path2points.begin() + 1, path2points.end());
  times.insert(times.end(), path2times.begin() + 1, path2times.end());
  

  // convert to a spline
  return convertSequentialStatesToSpline(metaSkeletonStateSpace,
                                         points, times);
}

::ompl::base::PathPtr planMinimumTimePath(
    const ::ompl::base::State* startState,
    const ::ompl::base::State* goalState,
    const ::ompl::base::SpaceInformationPtr& si,
    const DIMTPtr& dimt,
    double& pathCost,
    double maxPlanTime)
{
  ::ompl::base::PathPtr path = nullptr;
  pathCost = std::numeric_limits<double>::infinity();

  double singleSampleLimit = 3.0;
  double maxCallNum = 100;
  double batchSize = 100;
  int numTrials = 5;
  const double levelSet = std::numeric_limits<double>::infinity();

  ::ompl::geometric::MyInformedRRTstarPtr planner
      = ompl_make_shared<::ompl::geometric::MyInformedRRTstar>(si);

  // plan from start to via
  // 1. create problem
  ::ompl::base::ProblemDefinitionPtr basePdef
      = createProblem(si, startState, goalState);

  const ::ompl::base::OptimizationObjectivePtr baseOpt
      = createDimtOptimizationObjective(si, dimt, startState, goalState);
  basePdef->setOptimizationObjective(baseOpt);

  ::ompl::base::MyInformedSamplerPtr sampler
      = ompl_make_shared<::ompl::base::HitAndRunSampler>(
          si, basePdef, levelSet, maxCallNum, batchSize, numTrials);
  sampler->setSingleSampleTimelimit(singleSampleLimit);
  ::ompl::base::OptimizationObjectivePtr opt
      = ompl_make_shared<::ompl::base::MyOptimizationObjective>(
          si, sampler, startState, goalState);

  ::ompl::base::ProblemDefinitionPtr pdef
      = ompl_make_shared<::ompl::base::ProblemDefinition>(si);
  pdef->setStartAndGoalStates(startState, goalState);
  pdef->setOptimizationObjective(opt);

  // Set the problem instance for our planner to solve
  planner->setProblemDefinition(pdef);
  planner->setup();

  ::ompl::base::PlannerStatus status = planner->solve(maxPlanTime);

  if (status)
  {
    std::cout << "A solution is found" << std::endl;
    path = pdef->getSolutionPath();
    if(pdef->hasApproximateSolution())
    {
      path = planner->completeApproximateSolution(path);
    }
    if (path)
    {
      ::ompl::base::Cost path_cost = path->cost(opt);
      std::cout << "The cost is " << path_cost << std::endl;
      if (opt->isFinite(path_cost)==false)
      {
        return nullptr;
      }
      pathCost = path_cost.value();
    }
    else
    {
      return nullptr;
    }
  }
  else
  {
    return nullptr;
  }

  return path;
}

std::unique_ptr<aikido::trajectory::Spline> planMinimumTimeViaConstraint(
    const statespace::StateSpace::State* start,
    const statespace::StateSpace::State* goal,
    const statespace::StateSpace::State* via,
    const Eigen::VectorXd& viaVelocity,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const statespace::dart::MetaSkeletonStateSpacePtr& metaSkeletonStateSpace,
    const constraint::TestablePtr& validityConstraint,
    double& viaTime,
    double maxPlanTime,
    double maxDistanceBtwValidityChecks)
{
  std::size_t numDofs = metaSkeleton->getNumDofs();

  auto saver = MetaSkeletonStateSaver(
      metaSkeleton, MetaSkeletonStateSaver::Options::POSITIONS);
  DART_UNUSED(saver);

  // convert aikido state to Eigen::VectorXd
  Eigen::VectorXd startVec(numDofs);
  metaSkeletonStateSpace->logMap(start, startVec);
  Eigen::VectorXd goalVec(numDofs);
  metaSkeletonStateSpace->logMap(goal, goalVec);
  Eigen::VectorXd viaVec(numDofs);
  metaSkeletonStateSpace->logMap(via, viaVec);

  // Initialize parameters from bounds constraint
  std::vector<double> maxVelocities(numDofs, 0.0);
  std::vector<double> maxAccelerations(numDofs, 0.0);
  for (std::size_t i = 0; i < numDofs; i++)
  {
    double absUpperVel = std::abs(metaSkeleton->getVelocityUpperLimit(i));
    double absLowerVel = std::abs(metaSkeleton->getVelocityLowerLimit(i));
    maxVelocities[i] = absUpperVel > absLowerVel ? absLowerVel : absUpperVel;

    double absUpperAccl = std::abs(metaSkeleton->getAccelerationUpperLimit(i));
    double absLowerAccl = std::abs(metaSkeleton->getAccelerationLowerLimit(i));
    maxAccelerations[i]
        = absUpperAccl > absLowerAccl ? absLowerAccl : absUpperAccl;
  }
  DIMTPtr dimt
      = std::make_shared<DIMT>(numDofs, maxAccelerations, maxVelocities);

  // create bound constraint from metaSkeleton
  auto si = getSpaceInformation(
      dimt, metaSkeleton, validityConstraint, maxDistanceBtwValidityChecks);

  // create OMPL state
  Eigen::VectorXd startVel
      = Eigen::VectorXd::Zero(metaSkeletonStateSpace->getDimension());
  Eigen::VectorXd goalVel
      = Eigen::VectorXd::Zero(metaSkeletonStateSpace->getDimension());
  auto startState = allocState(si, startVec, startVel);
  auto goalState = allocState(si, goalVec, goalVel);
  auto viaState = allocState(si, viaVec, viaVelocity);

  double path1Duration = std::numeric_limits<double>::infinity();
  double path2Duration = std::numeric_limits<double>::infinity();
  ::ompl::base::PathPtr path1 = planMinimumTimePath(
      startState, viaState, si, dimt, path1Duration, maxPlanTime);

  ::ompl::base::PathPtr path2 = planMinimumTimePath(
      viaState, goalState, si, dimt, path2Duration, maxPlanTime);

  // concatenate two path
  // 1. create a vector of states and velocities
  // 2. push states/velocities of paths into the vector
  // 3. create SplineTrajectory from the vector
  auto traj = concatenateTwoPaths(path1, path2, dimt, 
                                  metaSkeletonStateSpace);
  if (traj != nullptr)
  {
    viaTime = path1Duration;
  }
  return traj;
}

} // namespace kinodynamics
} // namespace planner
} // namespace aikido
