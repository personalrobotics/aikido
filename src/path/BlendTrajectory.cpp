#include <aikido/path/BlendTrajectory.h>

namespace aikido {
namespace path {

BlendTrajectory::BlendTrajectory(
      const ConstTrajectoryPtr& _traj1,
      const ConstTrajectoryPtr& _traj2,
      double _timeStart,
      double _timeEnd)
  : mTrajectory1(_traj1),
    mTrajectory2(_traj2),
    mTimeStart(_timeStart),
    mTimeEnd(
      std::min(
        _timeEnd, std::min(
        _traj1->getDuration(),
        _traj2->getDuration())))
{
  assert(_timeStart <= _timeEnd);
  assert(_traj1->getNumOutputs() == _traj2->getNumOutputs());
}

auto BlendTrajectory::getNumOutputs() const -> Index
{
  return mTrajectory1->getNumOutputs();
}

auto BlendTrajectory::getNumDerivatives() const -> Index
{
  return std::max(
    mTrajectory1->getNumDerivatives(),
    mTrajectory2->getNumDerivatives());
}

auto BlendTrajectory::getDuration() const -> Scalar
{
  return mTrajectory2->getDuration();
}

auto BlendTrajectory::getBlendDuration() -> Scalar
{
  return mTimeEnd - mTimeStart;
}

Eigen::VectorXd BlendTrajectory::evaluate(Scalar _t, Index _derivative) const
{
  if (_t <= mTimeStart)
    return mTrajectory1->evaluate(_t, _derivative);
  else if (_t >= mTimeEnd)
    return mTrajectory2->evaluate(_t, _derivative);
  else
  {
    const double r = (_t - mTimeStart) / (mTimeEnd - mTimeStart);
    return (1 - r) * mTrajectory1->evaluate(_t, _derivative)
         + (    r) * mTrajectory2->evaluate(_t, _derivative);
  }
}

} // namespace path
} // namespace aikido
