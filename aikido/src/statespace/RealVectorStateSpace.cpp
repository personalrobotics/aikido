#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <boost/format.hpp>

using boost::format;
using boost::str;

namespace aikido {
namespace statespace {

//=============================================================================
RealVectorStateSpace::RealVectorStateSpace(int _dimension)
  : mDimension(_dimension)
{
  if (mDimension < 0)
    throw std::invalid_argument("_dimension must be positive.");
}

//=============================================================================
auto RealVectorStateSpace::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
Eigen::Map<Eigen::VectorXd> RealVectorStateSpace::getValue(State* _state) const
{
  auto valueBuffer = reinterpret_cast<double*>(
    reinterpret_cast<unsigned char*>(_state));

  return Eigen::Map<Eigen::VectorXd>(valueBuffer, mDimension);
}

//=============================================================================
Eigen::Map<const Eigen::VectorXd> RealVectorStateSpace::getValue(
  const State* _state) const
{
  auto valueBuffer = reinterpret_cast<const double*>(
    reinterpret_cast<const unsigned char*>(_state));

  return Eigen::Map<const Eigen::VectorXd>(valueBuffer, mDimension);
}

//=============================================================================
void RealVectorStateSpace::setValue(
  State* _state, const Eigen::VectorXd& _value) const
{
  getValue(_state) = _value;
}

//=============================================================================
size_t RealVectorStateSpace::getStateSizeInBytes() const
{
  return mDimension * sizeof(double);
}

//=============================================================================
StateSpace::State* RealVectorStateSpace::allocateStateInBuffer(
  void* _buffer) const
{
  auto state = reinterpret_cast<State*>(_buffer);
  getValue(state).setZero();
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

  getValue(out) = getValue(state1) + getValue(state2);
}

//=============================================================================
unsigned int RealVectorStateSpace::getDimension() const 
{
    return mDimension;
}

//=============================================================================
double RealVectorStateSpace::getMaximumExtent() const 
{
    
}

//=============================================================================
double RealVectorStateSpace::getMeasure() const 
{
    
}

//=============================================================================
bool RealVectorStateSpace::satisfiesBounds(const StateSpace::State* _state) const 
{
    
}

//=============================================================================
void RealVectorStateSpace::copyState(StateSpace::State* _destination,
                              const StateSpace::State* _source) const
{
    
}

//=============================================================================
double RealVectorStateSpace::distance(const StateSpace::State* _state1,
                               const StateSpace::State* _state2) const
{

}

//=============================================================================
bool RealVectorStateSpace::equalStates(const StateSpace::State* _state1,
                                const StateSpace::State* _state2) const
{
    
}

//=============================================================================
void RealVectorStateSpace::interpolate(const StateSpace::State* _from,
                                const StateSpace::State* _to,
                                const double _t,
                                StateSpace::State* _State) const
{

}


} // namespace statespace
} // namespace aikido
