#include <aikido/statespace/SO2StateSpace.hpp>

namespace aikido {
namespace statespace {

int SO2StateSpace::getRepresentationDimension() const
{
  return 1;
}


void SO2StateSpace::compose(const State& _state1, const State& _state2,
														State& _out) const
{
  const SO2State& state1 = static_cast<const SO2State&>(_state1);
  const SO2State& state2 = static_cast<const SO2State&>(_state2);

  SO2State& out = static_cast<SO2State&>(_out);
	out.mQ(0) = state1.mQ(0) + state2.mQ(0);
}	


Eigen::Isometry2d SO2StateSpace::SO2State::getIsometry() const
{
	Eigen::Rotation2D<double> rotation(mQ(0));
	Eigen::Isometry2d isometry(Eigen::Isometry2d::Identity());
	isometry.linear() = rotation.matrix();

  return isometry;
}

}
}
