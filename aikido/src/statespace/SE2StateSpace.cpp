#include <aikido/statespace/SE2StateSpace.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <Eigen/Geometry>

namespace aikido {
namespace statespace {

//=============================================================================
SE2StateSpace::State::State()
  : CompoundStateSpace::State({
      new SO2StateSpace::State,
      new RealVectorStateSpace::State(Eigen::Vector2d::Zero())
    })
{
}

//=============================================================================
SE2StateSpace::State::State(const Eigen::Isometry2d& _transform)
  : State()
{
  setIsometry(_transform);
}

//=============================================================================
Eigen::Isometry2d SE2StateSpace::State::getIsometry() const
{
  Eigen::Isometry2d out = Eigen::Isometry2d::Identity();

  const auto& orientation = static_cast<const SO2StateSpace::State&>(
    getState(0));
  out.rotate(orientation.getRotation());

  const auto& position = static_cast<const RealVectorStateSpace::State&>(
    getState(1));
  out.pretranslate(position.getValue().head<2>());

  return out;
}

//=============================================================================
void SE2StateSpace::State::setIsometry(const Eigen::Isometry2d& _transform)
{
  Eigen::Rotation2Dd rotation(0.);
  rotation.fromRotationMatrix(_transform.rotation());

  auto& orientation = static_cast<SO2StateSpace::State&>(getState(0));
  orientation.setRotation(rotation);

  auto& position = static_cast<RealVectorStateSpace::State&>(getState(1));
  position.setValue(_transform.translation());
}

//=============================================================================
SE2StateSpace::SE2StateSpace()
  : CompoundStateSpace({
      std::make_shared<SO2StateSpace>(),
      std::make_shared<RealVectorStateSpace>(2)
    })
{
}

} // namespace statespace
} // namespace aikido
