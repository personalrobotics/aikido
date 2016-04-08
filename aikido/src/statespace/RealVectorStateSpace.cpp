#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <dart/common/Console.h>

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
auto RealVectorStateSpace::getBounds() const -> const Bounds&
{
  return mBounds;
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

//=============================================================================
unsigned int RealVectorStateSpace::getDimension() const 
{
    return mBounds.rows();
}

//=============================================================================
double RealVectorStateSpace::getMaximumExtent() const 
{

    double d = 0.0;
    for (size_t i; i < mBounds.rows(); ++i)
    {
        double e = mBounds(i,1) - mBounds(i,0);
        d += e*e;
    }
    return std::sqrt(d);
}

//=============================================================================
double RealVectorStateSpace::getMeasure() const 
{
    double m = 1.0;
    for (size_t i = 0; i < mBounds.rows(); ++i)
    {
        m *= mBounds(i,1) - mBounds(i,0);
    }
    return m;
}

//=============================================================================
void RealVectorStateSpace::enforceBounds(StateSpace::State* _state) const 
{
    auto state = static_cast<State*>(_state);
    auto value = getMutableValue(state);
    
    for (size_t i = 0; i < mBounds.rows(); ++i)
    {
        if(value[i] > mBounds(i,1))
        {
            value[i] = mBounds(i,1);
        }
        else if(value[i] < mBounds(i,0))
        {
            value[i] = mBounds(i,0);
        }
    }
}

//=============================================================================
bool RealVectorStateSpace::satisfiesBounds(const StateSpace::State* _state) const 
{
    auto state = static_cast<const State*>(_state);
    auto value = getValue(state);

    for (size_t i = 0; i < mBounds.rows(); ++i)
    {
        if(value[i] - std::numeric_limits<double>::epsilon() > mBounds(i,1) ||
           value[i] + std::numeric_limits<double>::epsilon() < mBounds(i,0))
        {
            return false;
        }
    }
    return true;
}

//=============================================================================
void RealVectorStateSpace::copyState(StateSpace::State* _destination,
                                     const StateSpace::State* _source) const
{
    auto destination = static_cast<State*>(_destination);
    auto source = static_cast<const State*>(_source);
    setValue(destination, getValue(source));
}

//=============================================================================
double RealVectorStateSpace::distance(const StateSpace::State* _state1,
                                      const StateSpace::State* _state2) const
{
    auto v1 = getValue(static_cast<const State*>(_state1));
    auto v2 = getValue(static_cast<const State*>(_state2));

    double dist = 0.0;
    for (size_t i = 0; i < v1.size(); ++i)
    {
        double diff = v1[i] - v2[i];
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

//=============================================================================
bool RealVectorStateSpace::equalStates(const StateSpace::State* _state1,
                                const StateSpace::State* _state2) const
{
    return distance(_state1, _state2) < std::numeric_limits<double>::epsilon();
}

//=============================================================================
void RealVectorStateSpace::interpolate(const StateSpace::State* _from,
                                       const StateSpace::State* _to,
                                       const double _t,
                                       StateSpace::State* _state) const
{
    auto vfrom = getValue(static_cast<const State*>(_from));
    auto vto = getValue(static_cast<const State*>(_to));
    auto vstate = getMutableValue(static_cast<State*>(_state));
    for (size_t i = 0; i < vfrom.size(); ++i)
    {
        vstate[i] = vfrom[i] + (vto[i] - vfrom[i]) * _t;
    }
}

} // namespace statespace
} // namespace aikido
