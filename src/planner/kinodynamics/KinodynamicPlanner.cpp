#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "aikido/planner/kinodynamics/KinodynamicPlanner.hpp"
#include "aikido/planner/ompl/Planner.hpp"
#include "aikido/planner/kinodynamics/dimt/DoubleIntegratorMinimumTime.h"
#include "aikido/planner/kinodynamics/ompl/DimtStateSpace.hpp"
#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/planner/ompl/MotionValidator.hpp"
#include "aikido/planner/ompl/StateValidityChecker.hpp"
#include "aikido/planner/kinodynamics/ompl/MyOptimizationObjective.hpp"

namespace aikido {
namespace planner {
namespace kinodynamics {

::ompl::base::SpaceInformationPtr getSpaceInformation(
    DIMTPtr _dimt,
    statespace::StateSpacePtr _stateSpace,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    double _maxDistanceBtwValidityChecks)
{
  // construct the state space we are planning in
  ::ompl::base::StateSpacePtr space = std::make_shared< ::ompl::base::DimtStateSpace >(_dimt);

  ::ompl::base::RealVectorBounds bounds(_dimt->getNumDofs());
  bounds.setLow(-10);
  bounds.setHigh(10);
  space->as<::ompl::base::DimtStateSpace>()->setBounds(bounds);
  ::ompl::base::SpaceInformationPtr si = std::make_shared<::ompl::base::SpaceInformation>(space);

  // Validity checking
  std::vector<constraint::TestablePtr> constraints{
      std::move(_validityConstraint), std::move(_boundsConstraint)};
  auto conjunctionConstraint
      = std::make_shared<aikido::constraint::TestableIntersection>(
          std::move(_stateSpace), std::move(constraints));
  ::ompl::base::StateValidityCheckerPtr vchecker
      = std::make_shared<aikido::planner::ompl::StateValidityChecker>(si, conjunctionConstraint);
  si->setStateValidityChecker(vchecker);

  ::ompl::base::MotionValidatorPtr mvalidator
      = std::make_shared<aikido::planner::ompl::MotionValidator>(si, _maxDistanceBtwValidityChecks);
  si->setMotionValidator(mvalidator);
  si->setStateValidityCheckingResolution(0.001);
  si->setup();

  return si;
}

::ompl::base::ProblemDefinitionPtr createProblem(::ompl::base::SpaceInformationPtr si,
                                                 const ::ompl::base::State* start,
                                                 const ::ompl::base::State* goal)
{
  ::ompl::base::ScopedState<::ompl::base::RealVectorStateSpace> startState(si->getStateSpace(), start);
  ::ompl::base::ScopedState<::ompl::base::RealVectorStateSpace> goalState(si->getStateSpace(), goal);

  // Set up the final problem with the full optimization objective
  ::ompl::base::ProblemDefinitionPtr pdef = std::make_shared<::ompl::base::ProblemDefinition>(si);
  pdef->setStartAndGoalStates(startState, goalState);

  return pdef;
}

const ::ompl::base::OptimizationObjectivePtr createDimtOptimizationObjective(::ompl::base::SpaceInformationPtr si,
                                                                             DIMTPtr dimt,
                                                                             const ::ompl::base::State* start,
                                                                             const ::ompl::base::State* goal)
{
  ::ompl::base::ScopedState<::ompl::base::RealVectorStateSpace> startState(si->getStateSpace(), start);
  ::ompl::base::ScopedState<::ompl::base::RealVectorStateSpace> goalState(si->getStateSpace(), goal);

  const ::ompl::base::OptimizationObjectivePtr base_opt = std::make_shared<::ompl::base::DimtObjective>(si, startState, goalState, dimt);
  return base_opt;
}

trajectory::InterpolatedPtr planViaConstraint(
    const statespace::StateSpace::State* _start,
    const statespace::StateSpace::State* _goal,
    const statespace::StateSpace::State* _via,
    const Eigen::VectorXd& _viaVelocity,
    dart::dynamics::MetaSkeletonPtr _metaSkeleton,
    statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    double _maxPlanTime,
    double _maxDistanceBtwValidityChecks)
{
  std::size_t numDofs = _metaSkeleton->getNumDofs();
  // Initialize parameters from bounds constraint
  std::vector<double> maxVelocities(numDofs, 0.0);
  std::vector<double> maxAccelerations(numDofs, 0.0);
  for(std::size_t i=0; i<numDofs; i++)
  {
    double absUpperVel = std::abs(_metaSkeleton->getVelocityUpperLimit(i));
    double absLowerVel = std::abs(_metaSkeleton->getVelocityLowerLimit(i));
    maxVelocities[i] = absUpperVel>absLowerVel ? absLowerVel : absUpperVel;

    double absUpperAccl = std::abs(_metaSkeleton->getAccelerationUpperLimit(i));
    double absLowerAccl = std::abs(_metaSkeleton->getAccelerationLowerLimit(i));
    maxAccelerations[i] = absUpperAccl>absLowerAccl ? absLowerAccl : absUpperAccl;
  }
  DIMTPtr dimt = std::make_shared<DIMT>( numDofs, maxAccelerations, maxVelocities );

  auto si = getSpaceInformation(dimt, _metaSkeletonStateSpace,
                                _validityConstraint,
                                _boundsConstraint,
                                _maxDistanceBtwValidityChecks);


  // convert AIKIDO state to OMPL state

  // plan from start to via

  // 1. create problem
  //::ompl::base::ProblemDefinitionPtr basePdef1 = createProblem(si, _start, _via);
  //

  // plan from via to goal
  //::ompl::base::ProblemDefinitionPtr basePdef1 = createProblem(si, _start, _via);

  // concatenate two path







    return nullptr;
}

} // namespace kinodynamics
} // namespace planner
} // namespace aikido
