#include <aikido/planner/ompl/MotionValidator.hpp>

#include <ompl/base/SpaceInformation.h>
#include <aikido/common/StepSequence.hpp>
#include <aikido/common/VanDerCorput.hpp>

namespace aikido {
namespace planner {
namespace ompl {
MotionValidator::MotionValidator(
    const ::ompl::base::SpaceInformationPtr& _si,
    double _maxDistBtwValidityChecks)
  : ::ompl::base::MotionValidator(_si)
  , mSequenceResolution(_maxDistBtwValidityChecks)
{
  if (_si == nullptr)
  {
    throw std::invalid_argument("SpaceInformation is nullptr.");
  }

  if (mSequenceResolution <= 0)
  {
    throw std::invalid_argument(
        "Max distance between validity checks must be >= 0.");
  }
}

bool MotionValidator::checkMotion(
    const ::ompl::base::State* _s1, const ::ompl::base::State* _s2) const
{
  double dist = si_->distance(_s1, _s2);
  aikido::common::VanDerCorput vdc{1,
                                   true,
                                   true, // include endpoints
                                   mSequenceResolution / dist};

  auto stateSpace = si_->getStateSpace();
  auto iState = stateSpace->allocState();

  bool valid = true;
  for (double t : vdc)
  {
    stateSpace->interpolate(_s1, _s2, t, iState);
    if (!si_->isValid(iState))
    {
      valid = false;
      break;
    }
  }
  stateSpace->freeState(iState);
  return valid;
}

bool MotionValidator::checkMotion(
    const ::ompl::base::State* _s1,
    const ::ompl::base::State* _s2,
    std::pair<::ompl::base::State*, double>& _lastValid) const
{
  double dist = si_->distance(_s1, _s2);

  // Allocate a sequence that steps from 0 to 1 by a stepsize that ensures no
  // more than mSequenceResolution of distance between successive points
  aikido::common::StepSequence seq(
      mSequenceResolution / dist, true, true); // include endpoints

  auto stateSpace = si_->getStateSpace();
  auto iState = stateSpace->allocState();

  bool valid = true;
  double lastValidTime = 0.0;
  for (double t : seq)
  {
    stateSpace->interpolate(_s1, _s2, t, iState);
    if (!si_->isValid(iState))
    {
      valid = false;
      break;
    }
    lastValidTime = t;
  }
  stateSpace->freeState(iState);

  // Copy the last valid time and value into the return value
  _lastValid.second = lastValidTime;
  if (_lastValid.first)
  {
    stateSpace->interpolate(_s1, _s2, _lastValid.second, _lastValid.first);
  }

  return valid;
}
}
}
}
