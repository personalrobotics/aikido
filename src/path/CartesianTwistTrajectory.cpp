#include <dart/math/Geometry.h>
#include <aikido/path/CartesianTwistTrajectory.h>

namespace aikido {
namespace path {

SE3TwistTrajectory::SE3TwistTrajectory(const TrajectoryPtr& _twistTrajectory)
  : mTwistTrajectory(_twistTrajectory)
{
  assert(_twistTrajectory);
  assert(_twistTrajectory->getNumOutputs() == 6);
}

auto SE3TwistTrajectory::getNumDerivatives() const -> Index
{
  return 1;
}

auto SE3TwistTrajectory::getDuration() const -> Scalar
{
  if (mTwistTrajectory)
    return mTwistTrajectory->getDuration();
  else
    return 0.;
}

auto SE3TwistTrajectory::evaluate(Scalar _t) const -> Output
{
  if (mTwistTrajectory)
    return dart::math::expMap(mTwistTrajectory->evaluate(_t, 0));
  else
    return Output::Identity();
}

} // namespace path
} // namespace aikido
