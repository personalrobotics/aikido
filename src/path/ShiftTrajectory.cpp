#include <aikido/path/ShiftTrajectory.h>

namespace aikido {
namespace path {

ShiftTrajectory::ShiftTrajectory(const ConstTrajectoryPtr& _traj, Scalar _offset)
  : mTrajectory(_traj),
    mOffset(_offset)
{
  assert(_offset >= 0);
}

auto ShiftTrajectory::getNumOutputs() const -> Index
{
  return mTrajectory->getNumOutputs();
}

auto ShiftTrajectory::getNumDerivatives() const -> Index
{
  return mTrajectory->getNumDerivatives();
}

auto ShiftTrajectory::getDuration() const -> Scalar
{
  return std::max(mTrajectory->getDuration() - mOffset, 0.);

}

Eigen::VectorXd ShiftTrajectory::evaluate(Scalar _t, Index _derivative) const
{
  return mTrajectory->evaluate(_t + mOffset, _derivative);
}

} // namespace path
} // namespace aikido
