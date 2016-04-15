#include <aikido/ompl/OMPLPlanner.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>

namespace aikido
{
namespace ompl
{
::ompl::base::SpaceInformationPtr getSpaceInformation(
    const statespace::StateSpacePtr &_stateSpace,
    const statespace::InterpolatorPtr &_interpolator,
    const distance::DistanceMetricPtr &_dmetric,
    const constraint::SampleableConstraintPtr &_sampler,
    const constraint::TestableConstraintPtr &_boundsConstraint,
    const constraint::ProjectablePtr &_boundsProjector)
{
  // Ensure sampleable constraint and state space match
  if (_stateSpace != _sampler->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of sampler not equal to planning StateSpace");
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

  // Ensure sampler and state space match
  if (_stateSpace != _sampler->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of SampleableConstraint not equal to planning StateSpace");
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

  // AIKIDO State space
  auto sspace = boost::make_shared<AIKIDOGeometricStateSpace>(
    std::move(_stateSpace), std::move(_interpolator), std::move(_dmetric),
    std::move(_sampler), std::move(_boundsConstraint),
    std::move(_boundsProjector));

  // Space Information
  auto si = boost::make_shared<::ompl::base::SpaceInformation>(sspace);
  return si;
}

void setValidityConstraints(
    const ::ompl::base::SpaceInformationPtr &_si,
    const constraint::TestableConstraintPtr &_collConstraint,
    const constraint::TestableConstraintPtr &_boundsConstraint)
{
  // Validity checker
  std::vector<constraint::TestableConstraintPtr> constraints;
  constraints.push_back(_collConstraint);
  constraints.push_back(_boundsConstraint);
  ::ompl::base::StateValidityCheckerPtr vchecker =
      boost::make_shared<AIKIDOStateValidityChecker>(_si, constraints);
  _si->setStateValidityChecker(vchecker);
}
}
}
