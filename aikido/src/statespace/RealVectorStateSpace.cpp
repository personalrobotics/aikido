#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpaceSampleableConstraint.hpp>
#include <dart/common/Console.h>

#include <vector>
#include <random>
#include <aikido/constraint/Sampleable.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
RealVectorStateSpace::RealVectorStateSpace(int _dimension)
  : mBounds(_dimension, 2)
{
  if (_dimension < 0)
    throw std::invalid_argument("_dimension must be positive.");

  mBounds.col(0).setConstant(-std::numeric_limits<double>::infinity());
  mBounds.col(1).setConstant(+std::numeric_limits<double>::infinity());
}

//=============================================================================
RealVectorStateSpace::RealVectorStateSpace(const Bounds& _bounds)
  : mBounds(_bounds)
{
  for (std::size_t i = 0; i < mBounds.rows(); ++i)
  {
    if (mBounds(i, 0) > mBounds(i, 1))
    {
      std::stringstream msg;
      msg << "Lower bound exceeds upper bound for dimension "
          << i << ": " << mBounds(i, 0) << " > " << mBounds(i, 1) << ".";
      throw std::runtime_error(msg.str());
    }
  }
}

//=============================================================================
auto RealVectorStateSpace::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
Eigen::Map<Eigen::VectorXd> RealVectorStateSpace::getMutableValue(State* _state) const
{
  auto valueBuffer = reinterpret_cast<double*>(
    reinterpret_cast<unsigned char*>(_state));

  return Eigen::Map<Eigen::VectorXd>(valueBuffer, mBounds.rows());
}

//=============================================================================
Eigen::Map<const Eigen::VectorXd> RealVectorStateSpace::getValue(
  const State* _state) const
{
  auto valueBuffer = reinterpret_cast<const double*>(
    reinterpret_cast<const unsigned char*>(_state));

  return Eigen::Map<const Eigen::VectorXd>(valueBuffer, mBounds.rows());
}

//=============================================================================
int RealVectorStateSpace::getDimension() const
{
  return mBounds.rows();
}

//=============================================================================
auto RealVectorStateSpace::getBounds() const -> const Bounds&
{
  return mBounds;
}

//=============================================================================
constraint::SampleableConstraintPtr RealVectorStateSpace
  ::createSampleableConstraint(std::unique_ptr<util::RNG> _rng) const
{
  return std::make_shared<RealVectorStateSpaceSampleableConstraint>(
    // TODO: SampleableConstraint should operate on `const StateSpace`.
    std::const_pointer_cast<RealVectorStateSpace>(shared_from_this()),
    std::move(_rng));
}

//=============================================================================
void RealVectorStateSpace::setValue(
  State* _state, const Eigen::VectorXd& _value) const
{
  auto value = getMutableValue(_state);

  // TODO: Skip these checks in release mode.
  if (_value.size() != mBounds.rows())
  {
    std::stringstream msg;
    msg << "Value has incorrect size: expected " << mBounds.rows()
        << ", got " << _value.rows() << ".\n";
    throw std::runtime_error(msg.str());
  }

  for (size_t i = 0; i < _value.size(); ++i)
  {
    if (mBounds(i, 0) <= _value[i] && _value[i] <= mBounds(i, 1))
      value[i] = _value[i];
    else
      dtwarn << "[RealVectorStateSpace::setValue] Value " << _value[i]
             << " of dimension " << i << " is out of range ["
             << mBounds(i, 0) << ", " << mBounds(i, 0)
             << "]; ignoring this value.\n";
  }
}

//=============================================================================
size_t RealVectorStateSpace::getStateSizeInBytes() const
{
  return mBounds.rows() * sizeof(double);
}

//=============================================================================
StateSpace::State* RealVectorStateSpace::allocateStateInBuffer(
  void* _buffer) const
{
  auto state = reinterpret_cast<State*>(_buffer);
  getMutableValue(state).setZero();
  return state;
}

//=============================================================================
void RealVectorStateSpace::freeStateInBuffer(StateSpace::State* _state) const
{
  // Do nothing.
}

//=============================================================================
void RealVectorStateSpace::compose(
  const StateSpace::State* _state1, const StateSpace::State* _state2,
  StateSpace::State* _out) const
{
  auto state1 = static_cast<const State*>(_state1);
  auto state2 = static_cast<const State*>(_state2);
  auto out = static_cast<State*>(_out);

  getMutableValue(out) = getValue(state1) + getValue(state2);
}

} // namespace statespace
} // namespace aikido
