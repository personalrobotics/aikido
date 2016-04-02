#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <boost/format.hpp>

using boost::format;
using boost::str;

namespace aikido {
namespace statespace {

RealVectorStateSpace::RealVectorStateSpace(int _dimension)
: mDimension(_dimension)
{
  if (mDimension <= 0)
    throw std::invalid_argument("_dimension must be positive.");
}

int RealVectorStateSpace::getRepresentationDimension() const
{
  return mDimension;
}

void RealVectorStateSpace::compose(const State& _state1,
                                   const State& _state2, State& _out) const
{
  const RealVectorState& state1 = static_cast<const RealVectorState&>(_state1);
  const RealVectorState& state2 = static_cast<const RealVectorState&>(_state2);
  
  if (state1.mQ.rows() != state2.mQ.rows())
    throw std::invalid_argument("_state1 and state2 must have same dimension.");

  RealVectorState& out = static_cast<RealVectorState&>(_out);
  out.mQ = state1.mQ + state2.mQ;
  
}


}
}
