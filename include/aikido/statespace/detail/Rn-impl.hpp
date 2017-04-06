#include "aikido/statespace/Rn.hpp"

#include <type_traits>

namespace aikido {
namespace statespace {

/// \c StateHandle for a \c Rn. The template parameter is
/// necessary to support both \c const and non-<tt>const</tt> states.
///
/// \tparam _QualifiedState type of \c State being wrapped
template <class _QualifiedState>
class RealVectorStateHandle 
  : public statespace::StateHandle<Rn<_QualifiedState::Dimension>, _QualifiedState>
{
public:
  static constexpr int Dimension = _QualifiedState::Dimension;

  using VectorNd = typename Rn<Dimension>::VectorNd;

  using typename statespace::StateHandle<
  Rn<Dimension>, _QualifiedState>::State;
  using typename statespace::StateHandle<
    Rn<Dimension>, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<
    Rn<Dimension>, _QualifiedState>::QualifiedState;

  using ValueType = std::conditional<std::is_const<QualifiedState>::value,
    const VectorNd, VectorNd>;

  /// Construct and initialize to \c nullptr.
  RealVectorStateHandle()
  {
    // Do nothing
  }

  /// Construct a handle for \c _state in \c _space.
  ///
  /// \param _space state space that created \c _state
  /// \param _state state created by \c _space
  RealVectorStateHandle(const StateSpace* _space, QualifiedState* _state)
    : statespace::StateHandle<StateSpace, QualifiedState>(_space, _state)
  {
    // Do nothing
  }

  /// Gets the real vector stored in this state.
  ///
  /// \return real vector stored in this state
  Eigen::Map<const VectorNd> getValue()
  {
    return this->getStateSpace()->getValue(this->getState());
  }

  /// Sets the real vector stored in this state.
  ///
  /// \param _value real vector to store in \c _state
  void setValue(const VectorNd& _value)
  {
    return this->getStateSpace()->setValue(this->getState(), _value);
  }
};

//=============================================================================
template <int N>
Rn<N>::Rn()
{
  // Do nothing
}

//=============================================================================
template <int N>
auto Rn<N>::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
template <int N>
Eigen::Map<typename Rn<N>::VectorNd> Rn<N>::getMutableValue(State *_state) const
{
  auto valueBuffer =
      reinterpret_cast<double *>(reinterpret_cast<unsigned char *>(_state));

  return Eigen::Map<typename Rn<N>::VectorNd>(valueBuffer, mDimension);
}

//=============================================================================
template <int N>
Eigen::Map<const typename Rn<N>::VectorNd> Rn<N>::getValue(
    const State *_state) const
{
  auto valueBuffer = reinterpret_cast<const double *>(
      reinterpret_cast<const unsigned char *>(_state));

  return Eigen::Map<const typename Rn<N>::VectorNd>(valueBuffer, mDimension);
}

//=============================================================================
template <int N>
void Rn<N>::setValue(
    State *_state, const typename Rn<N>::VectorNd &_value) const
{
  // TODO: Skip this check in release mode.
  if (_value.size() != mDimension) {
    std::stringstream msg;
    msg << "Value has incorrect size: expected " << mDimension << ", got "
        << _value.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  getMutableValue(_state) = _value;
}

//=============================================================================
template <int N>
size_t Rn<N>::getStateSizeInBytes() const
{
  return mDimension * sizeof(double);
}

//=============================================================================
template <int N>
StateSpace::State *Rn<N>::allocateStateInBuffer(void *_buffer) const
{
  auto state = reinterpret_cast<State *>(_buffer);
  getMutableValue(state).setZero();
  return state;
}

//=============================================================================
template <int N>
void Rn<N>::freeStateInBuffer(StateSpace::State *_state) const
{
  // Do nothing.
}

//=============================================================================
template <int N>
void Rn<N>::compose(const StateSpace::State *_state1,
                 const StateSpace::State *_state2,
                 StateSpace::State *_out) const
{
  // TODO: Disable this in release mode.
  if (_state1 == _out || _state2 == _out)
    throw std::invalid_argument("Output aliases input.");

  auto state1 = static_cast<const State *>(_state1);
  auto state2 = static_cast<const State *>(_state2);
  auto out = static_cast<State *>(_out);

  setValue(out, getValue(state1) + getValue(state2));
}

//=============================================================================
template <int N>
size_t Rn<N>::getDimension() const
{
  return mDimension;
}

//=============================================================================
template <int N>
void Rn<N>::getIdentity(StateSpace::State *_out) const
{
  auto out = static_cast<State *>(_out);
  setValue(out, VectorNd::Zero());
}

//=============================================================================
template <int N>
void Rn<N>::getInverse(const StateSpace::State *_in, StateSpace::State *_out) const
{
  // TODO: Disable this in release mode.
  if (_out == _in)
    throw std::invalid_argument("Output aliases input.");

  auto in = static_cast<const State *>(_in);
  auto out = static_cast<State *>(_out);

  setValue(out, -getValue(in));
}

//=============================================================================
template <int N>
void Rn<N>::copyState(
    const StateSpace::State *_source, StateSpace::State *_destination) const
{
  auto destination = static_cast<State *>(_destination);
  auto source = static_cast<const State *>(_source);
  setValue(destination, getValue(source));
}

//=============================================================================
template <int N>
void Rn<N>::expMap(const Eigen::VectorXd &_tangent, StateSpace::State *_out) const
{
  // TODO: Skip this check in release mode.
  if (_tangent.size() != mDimension) {
    std::stringstream msg;
    msg << "Tangent vector has incorrect size: expected " << mDimension
        << ", got " << _tangent.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  auto out = static_cast<State *>(_out);
  setValue(out, _tangent);
}

//=============================================================================
template <int N>
void Rn<N>::logMap(const StateSpace::State *_in, Eigen::VectorXd &_tangent) const
{
  if (_tangent.size() != mDimension) {
    _tangent.resize(mDimension);
  }

  auto in = static_cast<const State *>(_in);
  _tangent = getValue(in);
}

//=============================================================================
template <int N>
void Rn<N>::print(const StateSpace::State *_state, std::ostream &_os) const
{
    auto val = getValue(static_cast<const State*>(_state));

    Eigen::IOFormat cleanFmt(3, Eigen::DontAlignCols, ",", ",", "", "", "[", "]");
    _os << val.format(cleanFmt);
}

} // namespace statespace
} // namespace aikido
