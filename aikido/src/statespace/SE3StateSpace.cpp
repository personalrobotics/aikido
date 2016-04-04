#include <aikido/statespace/SO3StateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
SE3StateSpace::State::State()
  : CompoundStateSpace::State({
      new SO3StateSpace::State,
      new RealVectorStateSpace::State(Eigen::Vector3d::Zero())
    })
{
}

//=============================================================================
SE3StateSpace::State::State(const Eigen::Isometry3d& _transform)
  : State()
{
  setIsometry(_transform);
}

//=============================================================================
Eigen::Isometry3d SE3StateSpace::State::getIsometry() const
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();

  const auto& orientation = static_cast<const SO3StateSpace::State&>(
    getState(0));
  out.rotate(orientation.getQuaternion());

  const auto& position = static_cast<const RealVectorStateSpace::State&>(
    getState(1));
  out.pretranslate(position.getValue().head<3>());

  return out;
}

//=============================================================================
void SE3StateSpace::State::setIsometry(const Eigen::Isometry3d& _transform)
{
  auto& orientation = static_cast<SO3StateSpace::State&>(getState(0));
  orientation.setQuaternion(Eigen::Quaterniond(_transform.rotation()));

  auto& position = static_cast<RealVectorStateSpace::State&>(getState(1));
  position.setValue(_transform.translation());
}

//=============================================================================
SE3StateSpace::SE3StateSpace()
  : CompoundStateSpace({
      std::make_shared<SO3StateSpace>(),
      std::make_shared<RealVectorStateSpace>(3)
    })
{
}

//=============================================================================
StateSpace::State* SE3StateSpace::allocateState() const
{
  return new State;
}

//=============================================================================
void SE3StateSpace::freeState(StateSpace::State* _state) const
{
  delete static_cast<State *>(_state);
}

} // namespace statespace
} // namespace aikido
