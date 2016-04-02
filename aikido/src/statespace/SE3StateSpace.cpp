#include <aikido/statespace/SE3StateSpace.hpp>
#include <dart/math/Geometry.h>

namespace aikido {
namespace statespace {

int SE3StateSpace::getRepresentationDimension() const
{
  return 6;
}


void SE3StateSpace::compose(const State& _state1, const State& _state2,
                            State& _out) const
{
  const SE3State& state1 = static_cast<const SE3State&>(_state1);
  const SE3State& state2 = static_cast<const SE3State&>(_state2);

  Eigen::Isometry3d isometry1 = state1.getIsometry();
  Eigen::Isometry3d isometry2 = state2.getIsometry();
  
  Eigen::Isometry3d isometry;
  isometry.matrix() = isometry1.matrix()*isometry2.matrix();
  
  SE3State& out = static_cast<SE3State&>(_out);
  out.mQ = ::dart::math::logMap(isometry);
}


Eigen::Isometry3d SE3StateSpace::SE3State::getIsometry() const
{
  return ::dart::math::expMap(mQ);
}


}
}
