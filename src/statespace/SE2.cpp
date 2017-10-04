#include <Eigen/Geometry>
#include <aikido/statespace/SE2.hpp>

namespace aikido {
namespace statespace {
//==============================================================================
SE2::State::State() : mTransform(Isometry2d::Identity())
{
}

//==============================================================================
SE2::State::State(const Isometry2d& _transform) : mTransform(_transform)
{
}

//==============================================================================
auto SE2::State::getIsometry() const -> const Isometry2d&
{
  return mTransform;
}

//==============================================================================
void SE2::State::setIsometry(const Isometry2d& _transform)
{
  mTransform = _transform;
}

//==============================================================================
auto SE2::createState() const -> ScopedState
{
  return ScopedState(this);
}

//==============================================================================
auto SE2::getIsometry(const State* _state) const -> const Isometry2d&
{
  return _state->getIsometry();
}

//==============================================================================
void SE2::setIsometry(State* _state, const Isometry2d& _transform) const
{
  _state->setIsometry(_transform);
}

//==============================================================================
size_t SE2::getStateSizeInBytes() const
{
  return sizeof(State);
}

//==============================================================================
StateSpace::State* SE2::allocateStateInBuffer(void* _buffer) const
{
  return new (_buffer) State;
}

//==============================================================================
void SE2::freeStateInBuffer(StateSpace::State* _state) const
{
  static_cast<State*>(_state)->~State();
}

//==============================================================================
void SE2::compose(
    const StateSpace::State* _state1,
    const StateSpace::State* _state2,
    StateSpace::State* _out) const
{
  // TODO: Disable this in release mode.
  if (_state1 == _out || _state2 == _out)
    throw std::invalid_argument("Output aliases input.");

  auto state1 = static_cast<const State*>(_state1);
  auto state2 = static_cast<const State*>(_state2);
  auto out = static_cast<State*>(_out);

  out->mTransform = state1->mTransform * state2->mTransform;
}

//==============================================================================
size_t SE2::getDimension() const
{
  return 3;
}

//==============================================================================
void SE2::getIdentity(StateSpace::State* _out) const
{
  auto out = static_cast<State*>(_out);
  setIsometry(out, Isometry2d::Identity());
}

//==============================================================================
void SE2::getInverse(
    const StateSpace::State* _in, StateSpace::State* _out) const
{
  // TODO: Disable this in release mode.
  if (_out == _in)
    throw std::invalid_argument("Output aliases input.");

  auto in = static_cast<const State*>(_in);
  auto out = static_cast<State*>(_out);
  setIsometry(out, getIsometry(in).inverse());
}

//==============================================================================
void SE2::copyState(
    const StateSpace::State* _source, StateSpace::State* _destination) const
{
  auto source = static_cast<const State*>(_source);
  auto dest = static_cast<State*>(_destination);
  setIsometry(dest, getIsometry(source));
}

//==============================================================================
void SE2::expMap(const Eigen::VectorXd& _tangent, StateSpace::State* _out) const
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
  Eigen::Vector2d translation = _tangent.tail<2>();

  Isometry2d transform(Isometry2d::Identity());
  transform.linear() = Eigen::Rotation2Dd(angle).matrix();
  transform.translation() = translation;

  out->mTransform = transform;
}

//==============================================================================
void SE2::logMap(const StateSpace::State* _in, Eigen::VectorXd& _tangent) const
{
  if (_tangent.rows() != 3)
    _tangent.resize(3);

  auto in = static_cast<const State*>(_in);

  Isometry2d transform = getIsometry(in);
  _tangent.tail<2>() = transform.translation();
  Eigen::Rotation2Dd rotation = Eigen::Rotation2Dd::Identity();
  rotation.fromRotationMatrix(transform.rotation());
  _tangent[0] = rotation.angle();
}

//==============================================================================
void SE2::print(const StateSpace::State* _state, std::ostream& _os) const
{
  auto state = static_cast<const State*>(_state);
  auto transform = getIsometry(state);

  Eigen::IOFormat cleanFmt(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",", "", "", "[", "]");
  Eigen::Rotation2Dd rotation = Eigen::Rotation2Dd::Identity();
  rotation.fromRotationMatrix(transform.rotation());
  _os << Eigen::Vector3d(
             transform.translation()[0],
             transform.translation()[1],
             rotation.angle())
             .format(cleanFmt);
}
} // namespace statespace
} // namespace aikido
