#include <aikido/statespace/SO3StateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
SE3StateSpace::SE3StateSpace()
  : CompoundStateSpace({
      std::make_shared<SO3StateSpace>(),
      std::make_shared<RealVectorStateSpace>(3)
    })
{
}

//=============================================================================
Eigen::Isometry3d SE3StateSpace::getIsometry(const State* _state) const
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.rotate(
    getSubStateHandle<SO3StateSpace>(_state, 0).getQuaternion());
  out.pretranslate(
    getSubStateHandle<RealVectorStateSpace>(_state, 1).getValue().head<3>());
  return out;
}

//=============================================================================
void SE3StateSpace::setIsometry(
  State* _state, const Eigen::Isometry3d& _transform) const
{
  getSubStateHandle<SO3StateSpace>(_state, 0).setQuaternion(
    Eigen::Quaterniond(_transform.rotation()));
  getSubStateHandle<RealVectorStateSpace>(_state, 1).setValue(
    _transform.translation());
}

} // namespace statespace
} // namespace aikido
