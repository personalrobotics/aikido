#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include "aikido/common/Spline.hpp"
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

using aikido::planner::ompl::ompl_make_shared;
using dart::dynamics::MetaSkeletonPtr;

namespace aikido {
namespace planner {
namespace kinodynamics {

class StateValidityChecker : public ompl::StateValidityChecker
{
public:
  StateValidityChecker(
      const ::ompl::base::SpaceInformationPtr& _si,
      constraint::TestablePtr _constraint)
    : ompl::StateValidityChecker(_si, _constraint)
  {
  }

  bool isValid(const ::ompl::base::State* _state) const override
  {
    if (!si_->getStateSpace()->satisfiesBounds(_state))
    {
      return false;
    }

    // create aikido state from ompl state
    auto stateSpace = mConstraint->getStateSpace();
    auto state = stateSpace->createState();
    Eigen::VectorXd config(stateSpace->getDimension());
    for (std::size_t i = 0; i < stateSpace->getDimension(); i++)
    {
      config[i] = _state->as<::ompl::base::RealVectorStateSpace::StateType>()
                      ->values[i];
    }
    stateSpace->expMap(config, state);
    return mConstraint->isSatisfied(state);
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
    DIMTPtr _dimt,
    MetaSkeletonPtr _skeleton,
    constraint::TestablePtr _validityConstraint,
    double _maxDistanceBtwValidityChecks)
{
  // construct the state space we are planning in
  ::ompl::base::StateSpacePtr space
      = ompl_make_shared<::ompl::base::DimtStateSpace>(_dimt);

  ::ompl::base::RealVectorBounds bounds(_skeleton->getNumDofs() * 2);
  for (std::size_t i = 0; i < _skeleton->getNumDofs(); i++)
  {
    bounds.setHigh(i, _skeleton->getPositionUpperLimit(i));
    bounds.setLow(i, _skeleton->getPositionLowerLimit(i));

    bounds.setHigh(
        _skeleton->getNumDofs() + i, _skeleton->getVelocityUpperLimit(i));
    bounds.setLow(
        _skeleton->getNumDofs() + i, _skeleton->getVelocityLowerLimit(i));
  }

  space->as<::ompl::base::DimtStateSpace>()->setBounds(bounds);
  ::ompl::base::SpaceInformationPtr si
      = ompl_make_shared<::ompl::base::SpaceInformation>(space);

  // Validity checking
  ::ompl::base::StateValidityCheckerPtr vchecker
      = ompl_make_shared<StateValidityChecker>(si, _validityConstraint);
  si->setStateValidityChecker(vchecker);

  ::ompl::base::MotionValidatorPtr mvalidator
      = ompl_make_shared<aikido::planner::ompl::MotionValidator>(
          si, _maxDistanceBtwValidityChecks);
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

std::unique_ptr<aikido::trajectory::Spline> planMinimumTimeViaConstraint(
    const statespace::StateSpace::State* _start,
    const statespace::StateSpace::State* _goal,
    const statespace::StateSpace::State* _via,
    const Eigen::VectorXd& _viaVelocity,
    dart::dynamics::MetaSkeletonPtr _metaSkeleton,
    statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    constraint::TestablePtr _validityConstraint,
    double& _viaTime,
    double _maxPlanTime,
    double _maxDistanceBtwValidityChecks)
{
  std::size_t numDofs = _metaSkeleton->getNumDofs();

  // convert aikido state to Eigen::VectorXd
  Eigen::VectorXd startVec(numDofs);
  _metaSkeletonStateSpace->logMap(_start, startVec);
  Eigen::VectorXd goalVec(numDofs);
  _metaSkeletonStateSpace->logMap(_goal, goalVec);
  Eigen::VectorXd viaVec(numDofs);
  _metaSkeletonStateSpace->logMap(_via, viaVec);

  // Initialize parameters from bounds constraint
  std::vector<double> maxVelocities(numDofs, 0.0);
  std::vector<double> maxAccelerations(numDofs, 0.0);
  for (std::size_t i = 0; i < numDofs; i++)
  {
    double absUpperVel = std::abs(_metaSkeleton->getVelocityUpperLimit(i));
    double absLowerVel = std::abs(_metaSkeleton->getVelocityLowerLimit(i));
    maxVelocities[i] = absUpperVel > absLowerVel ? absLowerVel : absUpperVel;

    double absUpperAccl = std::abs(_metaSkeleton->getAccelerationUpperLimit(i));
    double absLowerAccl = std::abs(_metaSkeleton->getAccelerationLowerLimit(i));
    maxAccelerations[i]
        = absUpperAccl > absLowerAccl ? absLowerAccl : absUpperAccl;
  }
  DIMTPtr dimt
      = std::make_shared<DIMT>(numDofs, maxAccelerations, maxVelocities);

  // create bound constraint from metaSkeleton
  auto si = getSpaceInformation(
      dimt, _metaSkeleton, _validityConstraint, _maxDistanceBtwValidityChecks);

  // create OMPL state
  Eigen::VectorXd startVel
      = Eigen::VectorXd::Zero(_metaSkeletonStateSpace->getDimension());
  Eigen::VectorXd goalVel
      = Eigen::VectorXd::Zero(_metaSkeletonStateSpace->getDimension());
  auto startState = allocState(si, startVec, startVel);
  auto goalState = allocState(si, goalVec, goalVel);
  auto viaState = allocState(si, viaVec, _viaVelocity);

  std::cout << "VALIDATE START " << si->isValid(startState) << std::endl;
  std::cout << "VALIDATE VIA " << si->isValid(viaState) << std::endl;
  std::cout << "VALIDATE GOAL " << si->isValid(goalState) << std::endl;

  double singleSampleLimit = 3.0;
  // double sigma = 1;
  // int max_steps = 20;
  // int max_steps = 2000;
  // double alpha = 0.5;
  double max_call_num = 100;
  double batch_size = 100;
  // double epsilon = 2;//0.2;
  // double L = 1;
  int num_trials = 5;
  const double level_set = std::numeric_limits<double>::infinity();

  ::ompl::geometric::MyInformedRRTstarPtr planner
      = ompl_make_shared<::ompl::geometric::MyInformedRRTstar>(si);

  // plan from start to via
  // 1. create problem
  ::ompl::base::ProblemDefinitionPtr basePdef1
      = createProblem(si, startState, viaState);

  const ::ompl::base::OptimizationObjectivePtr baseOpt1
      = createDimtOptimizationObjective(si, dimt, startState, viaState);
  basePdef1->setOptimizationObjective(baseOpt1);

  ::ompl::base::MyInformedSamplerPtr sampler1
      = ompl_make_shared<::ompl::base::HitAndRunSampler>(
          si, basePdef1, level_set, max_call_num, batch_size, num_trials);
  sampler1->setSingleSampleTimelimit(singleSampleLimit);
  ::ompl::base::OptimizationObjectivePtr opt1
      = ompl_make_shared<::ompl::base::MyOptimizationObjective>(
          si, sampler1, startState, viaState);

  ::ompl::base::ProblemDefinitionPtr pdef1
      = ompl_make_shared<::ompl::base::ProblemDefinition>(si);
  pdef1->setStartAndGoalStates(startState, viaState);
  pdef1->setOptimizationObjective(opt1);

  // Set the problem instance for our planner to solve
  planner->setProblemDefinition(pdef1);
  planner->setup();

  ::ompl::base::PlannerStatus solved = planner->solve(_maxPlanTime);

  ::ompl::base::PathPtr path1 = nullptr;
  if (pdef1->hasSolution())
  {
    std::cout << "First half has a solution" << std::endl;
    path1 = pdef1->getSolutionPath();
  }
  else
  {
    return nullptr;
  }

  // plan from via to goal
  // 1. create problem
  ::ompl::base::ProblemDefinitionPtr basePdef2
      = createProblem(si, viaState, goalState);

  const ::ompl::base::OptimizationObjectivePtr baseOpt2
      = createDimtOptimizationObjective(si, dimt, viaState, goalState);
  basePdef2->setOptimizationObjective(baseOpt2);

  ::ompl::base::MyInformedSamplerPtr sampler2
      = ompl_make_shared<::ompl::base::HitAndRunSampler>(
          si, basePdef2, level_set, max_call_num, batch_size, num_trials);
  sampler2->setSingleSampleTimelimit(singleSampleLimit);
  ::ompl::base::OptimizationObjectivePtr opt2
      = ompl_make_shared<::ompl::base::MyOptimizationObjective>(
          si, sampler2, viaState, goalState);

  ::ompl::base::ProblemDefinitionPtr pdef2
      = ompl_make_shared<::ompl::base::ProblemDefinition>(si);
  pdef2->setStartAndGoalStates(viaState, goalState);
  pdef2->setOptimizationObjective(opt2);

  // Set the problem instance for our planner to solve
  planner->setProblemDefinition(pdef2);
  planner->setup();

  solved = planner->solve(_maxPlanTime);

  ::ompl::base::PathPtr path2 = nullptr;
  if (pdef2->hasSolution())
  {
    std::cout << "Second half has a solution" << std::endl;
    path2 = pdef2->getSolutionPath();
  }
  else
  {
    return nullptr;
  }

  _viaTime = path1->cost(opt1).value();

  // concatenate two path
  // 1. create a vector of states and velocities
  // 2. push states/velocities of paths into the vector
  // 3. create SplineTrajectory from the vector

  double interpolateStepSize = 0.05;
  std::vector<Eigen::VectorXd> points;
  ::ompl::geometric::PathGeometric* geopath1
      = path1->as<::ompl::geometric::PathGeometric>();
  std::size_t node_num = geopath1->getStateCount();
  for (size_t idx = 0; idx < node_num - 1; idx++)
  {
    ::ompl::base::State* state1 = geopath1->getState(idx);
    ::ompl::base::State* state2 = geopath1->getState(idx + 1);
    std::vector<Eigen::VectorXd> deltaPoints
        = dimt->discretize(state1, state2, interpolateStepSize);
    points.insert(points.end(), deltaPoints.begin(), deltaPoints.end());
  }
  ::ompl::geometric::PathGeometric* geopath2
      = path2->as<::ompl::geometric::PathGeometric>();
  node_num = geopath2->getStateCount();
  for (size_t idx = 0; idx < node_num - 1; idx++)
  {
    ::ompl::base::State* state1 = geopath2->getState(idx);
    ::ompl::base::State* state2 = geopath2->getState(idx + 1);
    std::vector<Eigen::VectorXd> deltaPoints
        = dimt->discretize(state1, state2, interpolateStepSize);
    points.insert(points.end(), deltaPoints.begin(), deltaPoints.end());
  }

  std::size_t dimension = _metaSkeletonStateSpace->getDimension();
  using CubicSplineProblem
      = aikido::common::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

  auto _outputTrajectory
      = dart::common::make_unique<aikido::trajectory::Spline>(
          _metaSkeletonStateSpace);
  auto segmentStartState = _metaSkeletonStateSpace->createState();

  for (std::size_t i = 0; i < points.size() - 1; i++)
  {
    Eigen::VectorXd positionCurr = points[i].head(dimension);
    Eigen::VectorXd velocityCurr = points[i].tail(dimension);
    Eigen::VectorXd positionNext = points[i + 1].head(dimension);
    Eigen::VectorXd velocityNext = points[i + 1].tail(dimension);

    CubicSplineProblem problem(
        Eigen::Vector2d(i * interpolateStepSize, (i + 1) * interpolateStepSize),
        4,
        dimension);
    problem.addConstantConstraint(0, 0, positionCurr);
    problem.addConstantConstraint(0, 1, velocityCurr);
    problem.addConstantConstraint(1, 0, positionNext);
    problem.addConstantConstraint(1, 1, velocityNext);
    const auto spline = problem.fit();

    _metaSkeletonStateSpace->expMap(positionCurr, segmentStartState);

    // Add the ramp to the output trajectory.
    assert(spline.getCoefficients().size() == 1);
    const auto& coefficients = spline.getCoefficients().front();
    _outputTrajectory->addSegment(
        coefficients, interpolateStepSize, segmentStartState);
  }

  return _outputTrajectory;
}

} // namespace kinodynamics
} // namespace planner
} // namespace aikido
