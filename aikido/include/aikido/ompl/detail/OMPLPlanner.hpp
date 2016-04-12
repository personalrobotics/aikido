#include "../AIKIDOGeometricStateSpace.hpp"
#include "../AIKIDOStateValidityChecker.hpp"

namespace aikido {
namespace ompl {

template <class PlannerType>
void planOMPL(
    const aikido::statespace::StateSpace::State *_start,
    const aikido::statespace::StateSpace::State *_goal,
    const std::shared_ptr<aikido::statespace::StateSpace> &_stateSpace,
    const std::shared_ptr<aikido::constraint::TestableConstraint> &_constraint,
    const std::shared_ptr<aikido::distance::DistanceMetric> &_dmetric,
    const double &_maxPlanTime, std::unique_ptr<util::RNG> _rng) {

  // Ensure the constraint and state space match
  if (_stateSpace != _constraint->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to planning StateSpace");
  }

  // TODO: Ensure distance metric and state space match?

  // AIKIDO State space
  auto sspace = boost::make_shared<AIKIDOGeometricStateSpace>(std::move(_stateSpace),
                                                              std::move(_dmetric),
                                                              std::move(_rng));

  // Space Information
  auto si = boost::make_shared<::ompl::base::SpaceInformation>(sspace);

  // Validity checker
  std::vector<std::shared_ptr<aikido::constraint::TestableConstraint>>
      constraints;
  constraints.push_back(std::move(_constraint));
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

  if (solved) {
    // TODO: Extract trajectory
  }
}
}
}

