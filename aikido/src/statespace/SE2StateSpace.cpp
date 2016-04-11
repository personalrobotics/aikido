#include <aikido/statespace/SE2StateSpace.hpp>
#include <Eigen/Geometry>

namespace aikido {
namespace statespace {

//=============================================================================
SE2StateSpace::State::State()
  : mTransform(Isometry2d::Identity())
{
}

//=============================================================================
SE2StateSpace::State::State(const Isometry2d& _transform)
  : mTransform(_transform)
{
}

//=============================================================================
auto SE2StateSpace::State::getIsometry() const 
  -> const Isometry2d&
{
  return mTransform;
}

//=============================================================================
void SE2StateSpace::State::setIsometry(const Isometry2d& _transform)
{
  mTransform = _transform;
}

//=============================================================================
auto SE2StateSpace::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
auto SE2StateSpace::getIsometry(const State* _state) const
  -> const Isometry2d&
{
  return _state->getIsometry();
}

//=============================================================================
void SE2StateSpace::setIsometry(
  State* _state, const Isometry2d& _transform) const
{
  _state->setIsometry(_transform);
}


//=============================================================================
size_t SE2StateSpace::getStateSizeInBytes() const
{
  return sizeof(State);
}

//=============================================================================
StateSpace::State* SE2StateSpace::allocateStateInBuffer(void* _buffer) const
{
  return new (_buffer) State;
}

//=============================================================================
void SE2StateSpace::freeStateInBuffer(StateSpace::State* _state) const
{
  static_cast<State*>(_state)->~State();
}

//=============================================================================
void SE2StateSpace::compose(
  const StateSpace::State* _state1, const StateSpace::State* _state2,
  StateSpace::State* _out) const
{
  auto state1 = static_cast<const State*>(_state1);
  auto state2 = static_cast<const State*>(_state2);
  auto out = static_cast<State*>(_out);

  out->mTransform = state1->mTransform * state2->mTransform;
}

//=============================================================================
unsigned int SE2StateSpace::getDimension() const 
{
    return 3;
}

//=============================================================================
double SE2StateSpace::getMaximumExtent() const 
{

}

//=============================================================================
double SE2StateSpace::getMeasure() const 
{

}

//=============================================================================
void SE2StateSpace::copyState(StateSpace::State* _destination,
                              const StateSpace::State* _source) const
{

}

//=============================================================================
double SE2StateSpace::distance(const StateSpace::State* _state1,
                               const StateSpace::State* _state2) const
{

}

//=============================================================================
bool SE2StateSpace::equalStates(const StateSpace::State* _state1,
                                const StateSpace::State* _state2) const
{
    
}

//=============================================================================
void SE2StateSpace::interpolate(const StateSpace::State* _from,
                                const StateSpace::State* _to,
                                const double _t,
                                StateSpace::State* _State) const
{

}

//=============================================================================
void SE2StateSpace::expMap(
  const Eigen::VectorXd& _tangent, StateSpace::State* _out) const
{
  auto out = static_cast<State*>(_out);

  if (_tangent.rows() != 3)
  {
    std::stringstream msg;
    msg << "_tangent has incorrect size: expected 3"
        << ", got " << _tangent.rows() << ".\n";
    throw std::runtime_error(msg.str());
  }

  double angle = _tangent(0);
  Eigen::Vector2d translation = _tangent.bottomRows(2);

  Isometry2d transform(Isometry2d::Identity());
  transform.linear() = Eigen::Rotation2Dd(angle).matrix();
  transform.translation() = translation;

  out->mTransform = transform; 
}

} // namespace statespace
} // namespace aikido
