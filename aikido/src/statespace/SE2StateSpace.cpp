#include <aikido/statespace/SE2StateSpace.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <Eigen/Geometry>

namespace aikido {
namespace statespace {

//=============================================================================
SE2StateSpace::SE2StateSpace()
  : CompoundStateSpace({
      std::make_shared<SO2StateSpace>(),
      std::make_shared<RealVectorStateSpace>(2)
    })
{
}

//=============================================================================
Eigen::Isometry2d SE2StateSpace::getIsometry(const State* _state) const
{
  Eigen::Isometry2d out = Eigen::Isometry2d::Identity();
  out.rotate(
    getSubStateHandle<SO2StateSpace>(*_state, 0).getRotation());
  out.pretranslate(
    getSubStateHandle<RealVectorStateSpace>(*_state, 1).getValue().head<2>());
  return out;
}

//=============================================================================
void SE2StateSpace::setIsometry(
  State* _state, const Eigen::Isometry2d& _transform) const
{
  Eigen::Rotation2Dd rotation(0.);
  rotation.fromRotationMatrix(_transform.rotation());
  getSubStateHandle<SO2StateSpace>(*_state, 0).setRotation(rotation);
  getSubStateHandle<RealVectorStateSpace>(*_state, 1).setValue(
    _transform.translation());
}

} // namespace statespace
} // namespace aikido
