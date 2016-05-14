#include <aikido/control/BarrettFingerPositionCommandExecutor.hpp>
#include <thread>

namespace aikido{
namespace control{

//=============================================================================
BarrettFingerPositionCommandExecutor::BarrettFingerPositionCommandExecutor(
  ::dart::dynamics::ChainPtr _finger, int _primal, int _distal)
: mFinger(std::move(_finger))
, mPrimal (_primal)
, mDistal (_distal)
{
  if (!mFinger)
    throw std::invalid_argument("Finger is null.");

  if (!mFinger->getDof(mPrimal))
    throw std::invalid_argument("Finger does not have primal dof.");

  if (!mFinger->getDof(mDistal))
    throw std::invalid_argument("Finger does not have distal dof.");

  if (mDistal == mPrimal)
    throw std::invalid_argument("Primal and distal dofs should be different.");
}

//=============================================================================
double BarrettFingerPositionCommandExecutor::getMimicRatio() 
{
  return mimicRatio;
}

//=============================================================================
void BarrettFingerPositionCommandExecutor::execute(
  double _stepAngle, bool _distalOnly)
{
  if (_stepAngle == 0)
    return;

  if (!mFinger->isAssembled())
    throw std::runtime_error("Finger no longer linked.");

  // Move primal
  if (!_distalOnly){
    double currentAngle = mFinger->getDof(mPrimal)->getPosition();
    double nextAngle = _stepAngle + currentAngle;
    auto limits = mFinger->getDof(mPrimal)->getPositionLimits();
    if (nextAngle < limits.first)
      mFinger->getDof(mPrimal)->setPosition(limits.first);

    else if (nextAngle > limits.second)
      mFinger->getDof(mPrimal)->setPosition(limits.second);

    else
      mFinger->getDof(mPrimal)->setPosition(nextAngle);
  }

  // Move distal
  double currentAngle = mFinger->getDof(mDistal)->getPosition();
  double nextAngle = _stepAngle*mimicRatio + currentAngle;
  auto limits = mFinger->getDof(mDistal)->getPositionLimits();

  if (nextAngle < limits.first)
    mFinger->getDof(mDistal)->setPosition(limits.first);

  else if (nextAngle > limits.second)
    mFinger->getDof(mDistal)->setPosition(limits.second);

  else
    mFinger->getDof(mDistal)->setPosition(nextAngle);

}

}
}
