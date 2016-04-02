#include <aikido/statespace/SO3StateSpace.hpp>
#include <dart/math/Geometry.h>
#include <iostream>

namespace aikido {
namespace statespace {

int SO3StateSpace::getRepresentationDimension() const
{
  return 3;
}

void SO3StateSpace::compose(const State& _state1, const State& _state2,
                            State& _out) const
{
  const SO3State& state1 = static_cast<const SO3State&>(_state1);
  const SO3State& state2 = static_cast<const SO3State&>(_state2);

  Eigen::Isometry3d isometry1 = state1.getIsometry();
  Eigen::Isometry3d isometry2 = state2.getIsometry();

  Eigen::Isometry3d isometry;  
  isometry.matrix() = isometry1.matrix()*isometry2.matrix();

  SO3State& out = static_cast<SO3State&>(_out);
  out.mQ = ::dart::math::logMap(isometry).topRows(3);
  
}


Eigen::Isometry3d SO3StateSpace::SO3State::getIsometry() const
{
  Eigen::Vector6d s(Eigen::Vector6d::Zero());
  s.topRows(3) = mQ;
  Eigen::Isometry3d isometry = ::dart::math::expMap(s);

  return ::dart::math::expMap(s);
}


}
}
