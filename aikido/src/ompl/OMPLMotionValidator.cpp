#include <aikido/ompl/OMPLMotionValidator.hpp>
#include <aikido/util/VanDerCorput.hpp>
#include <aikido/util/StepSequence.hpp>
#include <ompl/base/SpaceInformation.h>

namespace aikido
{
namespace ompl
{
OMPLMotionValidator::OMPLMotionValidator(
    const ::ompl::base::SpaceInformationPtr& _si,
    const double& _maxDistBtwValidityChecks)
    : ::ompl::base::MotionValidator(_si)
    , mSequenceResolution(_maxDistBtwValidityChecks)
{
}

bool OMPLMotionValidator::checkMotion(const ::ompl::base::State* _s1,
                                      const ::ompl::base::State* _s2) const
{
  double dist = si_->distance(_s1, _s2);
  aikido::util::VanDerCorput vdc{1, true,  // include endpoints
                                 mSequenceResolution / dist};

  auto stateSpace = si_->getStateSpace();
  auto iState = stateSpace->allocState();

  bool valid = true;
  for (double t : vdc) {
    stateSpace->interpolate(_s1, _s2, t, iState);
    if (!si_->isValid(iState)) {
      valid = false;
      break;
    }
  }
  stateSpace->freeState(iState);
  return valid;
}

bool OMPLMotionValidator::checkMotion(
    const ::ompl::base::State* _s1, const ::ompl::base::State* _s2,
    std::pair<::ompl::base::State*, double>& _lastValid) const
{
  double dist = si_->distance(_s1, _s2);

  // Allocate a sequence that steps from 0 to 1 by a stepsize that ensures no
  // more than mSequenceResolution of distance between successive points
  aikido::util::StepSequence seq(mSequenceResolution / dist,
                                 true);  // include endpoints

  auto stateSpace = si_->getStateSpace();
  auto iState = stateSpace->allocState();

  bool valid = true;
  double lastValidTime = 0.0;
  for (double t : seq) {
    stateSpace->interpolate(_s1, _s2, t, iState);
    if (!si_->isValid(iState)) {
      valid = false;
      break;
    }
    lastValidTime = t;
  }
  stateSpace->freeState(iState);

  // Copy the last valid time and value into the return value
  _lastValid.second = lastValidTime;
  if (_lastValid.first) {
    stateSpace->interpolate(_s1, _s2, _lastValid.second, _lastValid.first);
  }

  return valid;
}
}
}
