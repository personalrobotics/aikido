#include "../AIKIDOGeometricStateSpace.hpp"
#include "../AIKIDOStateValidityChecker.hpp"
#include "../GoalRegion.hpp"
#include "../OMPLMotionValidator.hpp"
#include "../../path/PiecewiseLinearTrajectory.hpp"
#include <ompl/geometric/PathGeometric.h>

namespace aikido
{
namespace ompl
{
template <class PlannerType>
path::TrajectoryPtr planOMPL(const ::ompl::base::SpaceInformationPtr &_si,
                             const ::ompl::base::ProblemDefinitionPtr &_pdef,
                             const statespace::StateSpacePtr &_stateSpace,
                             const statespace::InterpolatorPtr &_interpolator,
                             const distance::DistanceMetricPtr &_dmetric,
                             const double &_maxPlanTime)
{
  // Planner
  ::ompl::base::PlannerPtr planner = boost::make_shared<PlannerType>(_si);
  planner->setProblemDefinition(_pdef);
  planner->setup();
  auto solved = planner->solve(_maxPlanTime);
  auto returnTraj = boost::make_shared<aikido::path::PiecewiseLinearTrajectory>(
    _stateSpace, _interpolator);

  if (solved) {
    // Get the path
    boost::shared_ptr<::ompl::geometric::PathGeometric> path =
        boost::static_pointer_cast<::ompl::geometric::PathGeometric>(
            _pdef->getSolutionPath());

    for (size_t idx = 0; idx < path->getStateCount(); ++idx) {
      const aikido::ompl::AIKIDOGeometricStateSpace::StateType *st =
          static_cast<aikido::ompl::AIKIDOGeometricStateSpace::StateType *>(
              path->getState(idx));
      // Arbitrary timing
      returnTraj->addWaypoint(idx, st->mState);
    }
  }
  return returnTraj;
}

template <class PlannerType>
path::TrajectoryPtr planOMPL(
    const statespace::StateSpace::State *_start,
    const statespace::StateSpace::State *_goal,
    const statespace::StateSpacePtr &_stateSpace,
    const statespace::InterpolatorPtr &_interpolator,
    const constraint::TestableConstraintPtr &_collConstraint,
    const constraint::TestableConstraintPtr &_boundsConstraint,
    const distance::DistanceMetricPtr &_dmetric,
    const constraint::SampleableConstraintPtr &_sampler,
    const constraint::ProjectablePtr &_boundsProjector,
    const double &_maxPlanTime,
    const double &_maxDistanceBtwValidityChecks)
{
  // Ensure the constraint and state space match
  if (_stateSpace != _collConstraint->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to planning StateSpace");
  }

  auto si = getSpaceInformation(_stateSpace, _interpolator, _dmetric, _sampler,
                                _boundsConstraint, _boundsProjector);

  // Validity
  setValidityConstraints(si, _collConstraint, _boundsConstraint);

  // Motion validation
  auto validator = boost::make_shared<OMPLMotionValidator>(si, _maxDistanceBtwValidityChecks);
  si->setMotionValidator(validator);

  // Start and states
  auto pdef = boost::make_shared<::ompl::base::ProblemDefinition>(si);
  auto sspace = boost::static_pointer_cast<AIKIDOGeometricStateSpace>(
      si->getStateSpace());
  auto start = sspace->allocState(_start);
  auto goal = sspace->allocState(_goal);
  pdef->setStartAndGoalStates(start, goal);

  return planOMPL<PlannerType>(si, pdef, _stateSpace, _interpolator, _dmetric, _maxPlanTime);
}

template <class PlannerType>
path::TrajectoryPtr planOMPL(
    const statespace::StateSpace::State *_start,
    const constraint::TestableConstraintPtr &_goalTestable,
    const constraint::SampleableConstraintPtr &_goalSampler,
    const statespace::StateSpacePtr &_stateSpace,
    const statespace::InterpolatorPtr &_interpolator,
    const constraint::TestableConstraintPtr &_collConstraint,
    const constraint::TestableConstraintPtr &_boundsConstraint,
    const distance::DistanceMetricPtr &_dmetric,
    const constraint::SampleableConstraintPtr &_sampler,
    const constraint::ProjectablePtr &_boundsProjector,
    const double &_maxPlanTime,
    const double &_maxDistanceBtwValidityChecks)
{
  // Ensure the constraint and state space match
  if (_stateSpace != _collConstraint->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to planning StateSpace");
  }

  auto si = getSpaceInformation(_stateSpace, _interpolator, _dmetric, _sampler,
                                _boundsConstraint, _boundsProjector);

  // Validity
  setValidityConstraints(si, _collConstraint, _boundsConstraint);

  // Motion validator
  auto validator = boost::make_shared<OMPLMotionValidator>(si, _maxDistanceBtwValidityChecks);
  si->setMotionValidator(validator);

  // Start and states
  auto pdef = boost::make_shared<::ompl::base::ProblemDefinition>(si);
  auto sspace = boost::static_pointer_cast<AIKIDOGeometricStateSpace>(
      si->getStateSpace());
  auto start = sspace->allocState(_start);
  pdef->addStartState(start);

  auto goalRegion = boost::make_shared<GoalRegion>(
      si, std::move(_goalTestable), _goalSampler->createSampleGenerator());
  pdef->setGoal(goalRegion);

  return planOMPL<PlannerType>(si, pdef, _stateSpace, _interpolator, _dmetric, _maxPlanTime);
}
}
}
