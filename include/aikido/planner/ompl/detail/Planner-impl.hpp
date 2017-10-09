#include <ompl/geometric/PathGeometric.h>
#include "../../../planner/ompl/BackwardCompatibility.hpp"
#include "../../../trajectory/Interpolated.hpp"
#include "../GeometricStateSpace.hpp"
#include "../GoalRegion.hpp"
#include "../StateValidityChecker.hpp"

namespace aikido {
namespace planner {
namespace ompl {

//==============================================================================
template <class PlannerType>
trajectory::InterpolatedPtr planOMPL(
    const statespace::StateSpace::State* _start,
    const statespace::StateSpace::State* _goal,
    statespace::StateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxPlanTime,
    double _maxDistanceBtwValidityChecks)
{

  if(!(_validityConstraint->isSatisfied(_start, true)))
    throw std::invalid_argument("Start State in Collision");

  // Create a SpaceInformation.  This function will ensure state space matching
  auto si = getSpaceInformation(
      _stateSpace,
      _interpolator,
      std::move(_dmetric),
      std::move(_sampler),
      std::move(_validityConstraint),
      std::move(_boundsConstraint),
      std::move(_boundsProjector),
      _maxDistanceBtwValidityChecks);

  // Start and states
  auto pdef = ompl_make_shared<::ompl::base::ProblemDefinition>(si);
  auto sspace
      = ompl_static_pointer_cast<GeometricStateSpace>(si->getStateSpace());
  auto start = sspace->allocState(_start);
  auto goal = sspace->allocState(_goal);

  // ProblemDefinition clones states and keeps them internally
  pdef->setStartAndGoalStates(start, goal);

  sspace->freeState(start);
  sspace->freeState(goal);

  auto planner = ompl_make_shared<PlannerType>(si);
  return planOMPL(
      planner,
      pdef,
      std::move(_stateSpace),
      std::move(_interpolator),
      _maxPlanTime);
}

//==============================================================================
template <class PlannerType>
trajectory::InterpolatedPtr planOMPL(
    const statespace::StateSpace::State* _start,
    constraint::TestablePtr _goalTestable,
    constraint::SampleablePtr _goalSampler,
    statespace::StateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxPlanTime,
    double _maxDistanceBtwValidityChecks)
{

  if(!(_validityConstraint->isSatisfied(_start, true)))
    throw std::invalid_argument("Start State in Collision");

  if (_goalTestable == nullptr)
  {
    throw std::invalid_argument("Testable goal is nullptr.");
  }

  if (_goalSampler == nullptr)
  {
    throw std::invalid_argument("Sampleable goal is nullptr.");
  }

  if (_goalTestable->getStateSpace() != _stateSpace)
  {
    throw std::invalid_argument("Testable goal does not match StateSpace");
  }

  if (_goalSampler->getStateSpace() != _stateSpace)
  {
    throw std::invalid_argument("Sampleable goal does not match StateSpace");
  }

  auto si = getSpaceInformation(
      _stateSpace,
      _interpolator,
      std::move(_dmetric),
      std::move(_sampler),
      std::move(_validityConstraint),
      std::move(_boundsConstraint),
      std::move(_boundsProjector),
      _maxDistanceBtwValidityChecks);

  // Set the start and goal
  auto pdef = ompl_make_shared<::ompl::base::ProblemDefinition>(si);
  auto sspace
      = ompl_static_pointer_cast<GeometricStateSpace>(si->getStateSpace());
  auto start = sspace->allocState(_start);
  pdef->addStartState(start); // copies
  sspace->freeState(start);

  auto goalRegion = ompl_make_shared<GoalRegion>(
      si, std::move(_goalTestable), _goalSampler->createSampleGenerator());
  pdef->setGoal(goalRegion);

  auto planner = ompl_make_shared<PlannerType>(si);
  return planOMPL(
      planner,
      pdef,
      std::move(_stateSpace),
      std::move(_interpolator),
      _maxPlanTime);
}

} // namespace ompl
} // namespace planner
} // namespace aikido
