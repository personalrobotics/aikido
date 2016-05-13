#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/planner/ompl/CRRT.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/planner/ompl/MotionValidator.hpp>
#include <aikido/constraint/TestableIntersection.hpp>

namespace aikido {
namespace planner {
namespace ompl {

//=============================================================================
::ompl::base::SpaceInformationPtr getSpaceInformation(
    statespace::StateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxDistanceBtwValidityChecks)
{
  if (_stateSpace == nullptr) {
    throw std::invalid_argument("StateSpace is nullptr.");
  }
  
  if (_interpolator == nullptr) {
      throw std::invalid_argument("Interpolator is nullptr.");
  }

  if (_dmetric == nullptr) {
      throw std::invalid_argument("DistanceMetric is nullptr.");
  }

  if (_sampler == nullptr) {
      throw std::invalid_argument("Sampler is nullptr.");
  }

  if (_validityConstraint == nullptr) {
      throw std::invalid_argument("ValidityConstraint is nullptr.");
  }

  if (_boundsConstraint == nullptr) {
      throw std::invalid_argument("BoundsConstraint is nullptr.");
  }

  if (_boundsProjector == nullptr) {
      throw std::invalid_argument("BoundsProjector is nullptr.");
  }

  if (_stateSpace != _interpolator->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of interpolator not equal to planning StateSpace");
  }

  // Ensure distance metric and state space match
  if (_stateSpace != _dmetric->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of DistanceMetric not equal to planning StateSpace");
  }

  // Ensure sampleable constraint and state space match
  if (_stateSpace != _sampler->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of sampler not equal to planning StateSpace");
  }

  // Ensure bounds constraint and state space match
  if (_stateSpace != _validityConstraint->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of ValidityConstraint not equal to planning StateSpace");
  }

  // Ensure bounds constraint and state space match
  if (_stateSpace != _boundsConstraint->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of BoundsConstraint not equal to planning StateSpace");
  }

  // Ensure the projector state space and state space match
  if (_stateSpace != _boundsProjector->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of BoundsProjector not equal to planning StateSpace");
  }

  // Ensure max distance between validity checks is positive
  if (_maxDistanceBtwValidityChecks <= 0.0) {
      throw std::invalid_argument(
          "Max distance between validity checks must be >= 0");
  }

  // Geometric State space
  auto sspace = boost::make_shared<GeometricStateSpace>(
    _stateSpace, std::move(_interpolator), std::move(_dmetric),
    std::move(_sampler), _boundsConstraint,
    std::move(_boundsProjector));

  // Space Information
  auto si = boost::make_shared<::ompl::base::SpaceInformation>(std::move(sspace));

  // Validity checking
  std::vector<constraint::TestablePtr> constraints{
      std::move(_validityConstraint), std::move(_boundsConstraint)};
  auto conjunctionConstraint =
      std::make_shared<constraint::TestableIntersection>(std::move(_stateSpace),
                                                          std::move(constraints));
  ::ompl::base::StateValidityCheckerPtr vchecker =
      boost::make_shared<StateValidityChecker>(si, conjunctionConstraint);
  si->setStateValidityChecker(vchecker);

  ::ompl::base::MotionValidatorPtr mvalidator =
        boost::make_shared<MotionValidator>(si, _maxDistanceBtwValidityChecks);
  si->setMotionValidator(mvalidator);

  return si;
}

//=============================================================================
trajectory::InterpolatedPtr planOMPL(
  const ::ompl::base::PlannerPtr &_planner,
  const ::ompl::base::ProblemDefinitionPtr &_pdef,
  statespace::StateSpacePtr _sspace,
  statespace::InterpolatorPtr _interpolator,
  double _maxPlanTime)
{
  _planner->setProblemDefinition(_pdef);
  _planner->setup();
  auto solved = _planner->solve(_maxPlanTime);

  if (solved) {
    auto returnTraj = std::make_shared<trajectory::Interpolated>(
        std::move(_sspace), std::move(_interpolator));

    // Get the path
    auto path =
        boost::dynamic_pointer_cast<::ompl::geometric::PathGeometric>(
            _pdef->getSolutionPath());
    if(!path){
      throw std::invalid_argument(
          "Path is not of type PathGeometric. Cannot convert to aikido "
          "Trajectory");
    }

    for (size_t idx = 0; idx < path->getStateCount(); ++idx) {
      const auto *st =
          static_cast<GeometricStateSpace::StateType *>(
              path->getState(idx));
      // Arbitrary timing
      returnTraj->addWaypoint(idx, st->mState);
    }

    return returnTraj;
  }
  return nullptr;
}

//=============================================================================
trajectory::InterpolatedPtr planCRRT(
    const statespace::StateSpace::State *_start,
    constraint::TestablePtr _goalTestable,
    constraint::SampleablePtr _goalSampler,
    constraint::ProjectablePtr _trajConstraint,
    statespace::StateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxPlanTime, double _maxExtensionDistance,
    double _maxDistanceBtwProjections)
{
  if (_goalTestable == nullptr) {
    throw std::invalid_argument("Testable goal is nullptr.");
  }

  if (_goalSampler == nullptr) {
    throw std::invalid_argument("Sampleable goal is nullptr.");
  }

  if (_trajConstraint == nullptr) {
    throw std::invalid_argument("Trajectory constraint is nullptr.");
  }

  if (_goalTestable->getStateSpace() != _stateSpace) {
    throw std::invalid_argument("Testable goal does not match StateSpace");
  }

  if (_goalSampler->getStateSpace() != _stateSpace) {
    throw std::invalid_argument("Sampleable goal does not match StateSpace");
  }

  if (_trajConstraint->getStateSpace() != _stateSpace) {
    throw std::invalid_argument(
        "Trajectory constraint does not match StateSpace");
  }

  auto si = getSpaceInformation(
      _stateSpace, _interpolator, std::move(_dmetric), std::move(_sampler),
      std::move(_validityConstraint), std::move(_boundsConstraint),
      std::move(_boundsProjector), _maxDistanceBtwProjections);

  // Set the start and goal
  auto pdef = boost::make_shared<::ompl::base::ProblemDefinition>(si);
  auto sspace = boost::static_pointer_cast<GeometricStateSpace>(
      si->getStateSpace());
  auto start = sspace->allocState(_start);
  pdef->addStartState(start); // copies
  sspace->freeState(start);

  auto goalRegion = boost::make_shared<GoalRegion>(
      si, std::move(_goalTestable), _goalSampler->createSampleGenerator());
  pdef->setGoal(goalRegion);

  auto planner = boost::make_shared<CRRT>(si);
  planner->setPathConstraint(std::move(_trajConstraint));
  planner->setRange(_maxExtensionDistance);
  planner->setProjectionResolution(_maxDistanceBtwProjections);
  return planOMPL(planner, pdef, std::move(_stateSpace),
                  std::move(_interpolator), _maxPlanTime);
}

}
}
}
