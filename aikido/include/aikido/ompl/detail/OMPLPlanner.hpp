#include "../AIKIDOGeometricStateSpace.hpp"
#include "../AIKIDOStateValidityChecker.hpp"
#include "../../path/PiecewiseLinearTrajectory.hpp"
#include <ompl/geometric/PathGeometric.h>

namespace aikido
{
namespace ompl
{
template <class PlannerType>
aikido::path::TrajectoryPtr planOMPL(
    const aikido::statespace::StateSpace::State *_start,
    const aikido::statespace::StateSpace::State *_goal,
    const std::shared_ptr<aikido::statespace::StateSpace> &_stateSpace,
    const std::shared_ptr<aikido::constraint::TestableConstraint> &_collConstraint,
    std::unique_ptr<aikido::constraint::TestableConstraint> _boundsConstraint,
    const aikido::distance::DistanceMetricPtr &_dmetric,
    std::unique_ptr<aikido::constraint::SampleableConstraint> _sampler,
    const double &_maxPlanTime)
{
  // Ensure the constraint and state space match
  if (_stateSpace != _collConstraint->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to planning StateSpace");
  }

  // Ensure sampleable constraint and state space match
  if (_stateSpace != _sampler->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of sampler not equal to planning StateSpace");
  }

  // TODO: Ensure distance metric and state space match?

  // AIKIDO State space
  auto sspace = boost::make_shared<AIKIDOGeometricStateSpace>(
      std::move(_stateSpace), std::move(_dmetric), std::move(_sampler),
      std::move(_boundsConstraint));

  // Space Information
  auto si = boost::make_shared<::ompl::base::SpaceInformation>(sspace);

  // Validity checker
  std::vector<std::shared_ptr<aikido::constraint::TestableConstraint>>
      constraints;
  constraints.push_back(std::move(_collConstraint));
  ::ompl::base::StateValidityCheckerPtr vchecker =
      boost::make_shared<AIKIDOStateValidityChecker>(si, constraints);
  si->setStateValidityChecker(vchecker);

  // Start and states
  auto pdef = boost::make_shared<::ompl::base::ProblemDefinition>(si);
  auto start = sspace->allocState(_start);
  auto goal = sspace->allocState(_goal);
  pdef->setStartAndGoalStates(start, goal);

  // Planner
  ::ompl::base::PlannerPtr planner = boost::make_shared<PlannerType>(si);
  planner->setProblemDefinition(pdef);
  planner->setup();
  auto solved = planner->solve(_maxPlanTime);
  
  boost::shared_ptr<aikido::path::PiecewiseLinearTrajectory> returnTraj =
      boost::make_shared<aikido::path::PiecewiseLinearTrajectory>(_stateSpace,
                                                                _dmetric);

  if (solved) {
      // Get the path
      boost::shared_ptr<::ompl::geometric::PathGeometric> path = 
          boost::static_pointer_cast<::ompl::geometric::PathGeometric>(pdef->getSolutionPath());

      for (size_t idx = 0; idx < path->getStateCount(); ++idx) {
          const aikido::ompl::AIKIDOGeometricStateSpace::StateType* st = 
              static_cast<aikido::ompl::AIKIDOGeometricStateSpace::StateType*>(path->getState(idx));
          // Arbitrary timing
          returnTraj->addWaypoint(idx, st->mState);
      }
  }

  return returnTraj;
}
}
}
