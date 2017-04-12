#include "aikido/statespace/Rn.hpp"

#include <type_traits>

namespace aikido {
namespace statespace {

//==============================================================================
extern template
class R<0>;

extern template
class R<1>;

extern template
class R<2>;

extern template
class R<3>;

extern template
class R<6>;

extern template
class R<Eigen::Dynamic>;

//==============================================================================
/// \c StateHandle for a \c Rn. The template parameter is
/// necessary to support both \c const and non-<tt>const</tt> states.
///
/// \tparam _QualifiedState type of \c State being wrapped
template <class _QualifiedState>
class RStateHandle
  : public statespace::StateHandle<R<_QualifiedState::DimensionAtCompileTime>, _QualifiedState>
{
public:
  static constexpr int DimensionAtCompileTime = _QualifiedState::DimensionAtCompileTime;

  using Vectord = typename R<DimensionAtCompileTime>::Vectord;

  using typename statespace::StateHandle<
  R<DimensionAtCompileTime>, _QualifiedState>::State;
  using typename statespace::StateHandle<
    R<DimensionAtCompileTime>, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<
    R<DimensionAtCompileTime>, _QualifiedState>::QualifiedState;

  using ValueType = std::conditional<std::is_const<QualifiedState>::value,
    const Vectord, Vectord>;

  /// Construct and initialize to \c nullptr.
  RStateHandle()
  {
    // Do nothing
  }

  /// Construct a handle for \c _state in \c _space.
  ///
  /// \param _space state space that created \c _state
  /// \param _state state created by \c _space
  RStateHandle(const StateSpace* _space, QualifiedState* _state)
    : statespace::StateHandle<StateSpace, QualifiedState>(_space, _state)
  {
    // Do nothing
  }

  /// Gets the real vector stored in this state.
  ///
  /// \return real vector stored in this state
  Eigen::Map<const Vectord> getValue()
  {
    return this->getStateSpace()->getValue(this->getState());
  }

  /// Sets the real vector stored in this state.
  ///
  /// \param _value real vector to store in \c _state
  void setValue(const Vectord& _value)
  {
    return this->getStateSpace()->setValue(this->getState(), _value);
  }
};

//=============================================================================
template <int N>
R<N>::R()
{
  static_assert(N > -2,
      "Invalid dimension. The dimension should be non-negative.");

  if (N == Eigen::Dynamic)
    mDimension = 0;
}

//=============================================================================
template <int N>
R<N>::R(int dimension) : mDimension(dimension)
{
  static_assert(N > -2,
      "Invalid dimension. The dimension should be either -1 for dynamic size "
      "state space or non-negative for fixed size state space.");

  if (N != Eigen::Dynamic)
  {
    std::stringstream msg;
    msg << "Invalid constructor. You called a constructor for fixed size "
        << "state space on dynamic size state space.";
    throw std::invalid_argument(msg.str());
  }
}

//=============================================================================
template <int N>
auto R<N>::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
template <int N>
auto R<N>::getMutableValue(State *_state) const -> Eigen::Map<Vectord>
{
  auto valueBuffer =
      reinterpret_cast<double *>(reinterpret_cast<unsigned char *>(_state));

  return Eigen::Map<Vectord>(valueBuffer, getDimension());
}

//=============================================================================
template <int N>
auto R<N>::getValue(const State *_state) const -> Eigen::Map<const Vectord>
{
  auto valueBuffer = reinterpret_cast<const double *>(
      reinterpret_cast<const unsigned char *>(_state));

  return Eigen::Map<const Vectord>(valueBuffer, getDimension());
}

//=============================================================================
template <int N>
void R<N>::setValue(State *_state, const typename R<N>::Vectord &_value) const
{
  // TODO: Skip this check in release mode.
  if (_value.size() != getDimension()) {
    std::stringstream msg;
    msg << "Value has incorrect size: expected " << getDimension() << ", got "
        << _value.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  getMutableValue(_state) = _value;
}

//=============================================================================
template <int N>
size_t R<N>::getStateSizeInBytes() const
{
  return getDimension() * sizeof(double);
}

//=============================================================================
template <int N>
StateSpace::State *R<N>::allocateStateInBuffer(void *_buffer) const
{
  auto state = reinterpret_cast<State *>(_buffer);
  getMutableValue(state).setZero();
  return state;
}

//=============================================================================
template <int N>
void R<N>::freeStateInBuffer(StateSpace::State */*_state*/) const
{
  // Do nothing.
}

//=============================================================================
template <int N>
void R<N>::compose(const StateSpace::State *_state1,
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
size_t R<N>::getDimension() const
{
  if (N == Eigen::Dynamic)
    return mDimension;
  else
    return N;
}

//=============================================================================
template <int N>
void R<N>::getIdentity(StateSpace::State *_out) const
{
  auto out = static_cast<State *>(_out);

  setValue(out, Vectord::Zero(getDimension()));
}

//=============================================================================
template <int N>
void R<N>::getInverse(const StateSpace::State *_in, StateSpace::State *_out) const
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
void R<N>::copyState(
    const StateSpace::State *_source, StateSpace::State *_destination) const
{
  auto destination = static_cast<State *>(_destination);
  auto source = static_cast<const State *>(_source);
  setValue(destination, getValue(source));
}

//=============================================================================
template <int N>
void R<N>::expMap(const Eigen::VectorXd&_tangent, StateSpace::State *_out) const
{
  // TODO: Skip this check in release mode.
  if (_tangent.size() != getDimension()) {
    std::stringstream msg;
    msg << "Tangent vector has incorrect size: expected " << getDimension()
        << ", got " << _tangent.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  auto out = static_cast<State *>(_out);
  setValue(out, _tangent);
}

//=============================================================================
template <int N>
void R<N>::logMap(const StateSpace::State *_in, Eigen::VectorXd& _tangent) const
{
  if (_tangent.size() != getDimension())
    _tangent.resize(getDimension());

  auto in = static_cast<const State *>(_in);
  _tangent = getValue(in);
}

//=============================================================================
template <int N>
void R<N>::print(const StateSpace::State *_state, std::ostream &_os) const
{
  auto val = getValue(static_cast<const State*>(_state));

  Eigen::IOFormat cleanFmt(3, Eigen::DontAlignCols, ",", ",", "", "", "[", "]");
  _os << val.format(cleanFmt);
}

} // namespace statespace
} // namespace aikido
