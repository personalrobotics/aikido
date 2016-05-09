#include <aikido/control/FingerSimulationStepExecutor.hpp>
#include <thread>

namespace aikido{
namespace control{

//=============================================================================
FingerSimulationStepExecutor::FingerSimulationStepExecutor(
  ::dart::dynamics::ChainPtr _finger)
: mFinger(std::move(_finger))
{
  if (!mFinger)
    throw std::invalid_argument("Finger is null.");

  if (mFinger->getNumDofs() != 2)
  {
    std::stringstream msg;
    msg << "Finger should have 2 Dofs, has " << mFinger->getNumDofs()
    << " Dofs. ";
    throw std::invalid_argument(msg.str());
  }

}

//=============================================================================
double FingerSimulationStepExecutor::getMimicRatio() 
{
  return mimicRatio;
}

//=============================================================================
void FingerSimulationStepExecutor::execute(double _stepAngle, bool _distalOnly)
{
  if (_stepAngle == 0)
    return;

  // Move primal
  if (!_distalOnly){
    double currentAngle = mFinger->getDof(0)->getPosition();
    double nextAngle = _stepAngle + currentAngle;
    auto limits = mFinger->getDof(0)->getPositionLimits();
    if (nextAngle < limits.first)
      mFinger->getDof(0)->setPosition(limits.first);

    else if (nextAngle > limits.second)
      mFinger->getDof(0)->setPosition(limits.second);

    else
      mFinger->getDof(0)->setPosition(nextAngle);
  }

  // Move distal
  double currentAngle = mFinger->getDof(1)->getPosition();
  double nextAngle = _stepAngle*mimicRatio + currentAngle;
  auto limits = mFinger->getDof(1)->getPositionLimits();

  if (nextAngle < limits.first)
    mFinger->getDof(1)->setPosition(limits.first);

  else if (nextAngle > limits.second)
    mFinger->getDof(1)->setPosition(limits.second);

  else
    mFinger->getDof(1)->setPosition(nextAngle);

}

}
}
