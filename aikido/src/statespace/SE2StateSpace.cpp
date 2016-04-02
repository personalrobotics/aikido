#include <aikido/statespace/SE2StateSpace.hpp>
#include <Eigen/Geometry>

namespace aikido {
namespace statespace {


int SE2StateSpace::getRepresentationDimension() const
{
  return 3;
}

void SE2StateSpace::compose(const State& _state1, const State& _state2,
                            State& _out) const
{
  const SE2State& state1 = static_cast<const SE2State&>(_state1);
  const SE2State& state2 = static_cast<const SE2State&>(_state2);

  Eigen::Isometry2d isometry1 = state1.getIsometry();
  Eigen::Isometry2d isometry2 = state2.getIsometry();

  Eigen::Isometry2d isometry;
  isometry.matrix() = isometry1.matrix()*isometry2.matrix();

  Eigen::Rotation2D<double> rotation;
  rotation = rotation.fromRotationMatrix(isometry.linear());

  SE2State& out = static_cast<SE2State&>(_out);
  out.mQ(0) = rotation.angle();
  out.mQ.bottomRows(2) = isometry.translation();
  
}

Eigen::Isometry2d SE2StateSpace::SE2State::getIsometry() const
{
  Eigen::Isometry2d isometry(Eigen::Isometry2d::Identity());
  isometry.translation() = mQ.bottomRows(2);
  isometry.linear() = Eigen::Rotation2D<double>(mQ(0)).matrix();

  return isometry;
}


}
}
