#ifndef AIKIDO_STATESPACE_REALVECTORSTATESPACE_H
#define AIKIDO_STATESPACE_REALVECTORSTATESPACE_H

#include "StateSpace.hpp"
#include "State.hpp"
#include "Jacobian.hpp"

namespace aikido {
namespace statespace {

class RealVectorStateSpace : public StateSpace
{
public:
  using RealVectorState = UtilState;
  using RealVectorJacobian = UtilJacobian;

  RealVectorStateSpace(int _dimension);

  int getRepresentationDimension() const override;

  void compose(const State& _state1, const State& _state2,
               State& _out) const override;

private:
  int mDimension;
};

using RealVectorState = RealVectorStateSpace::RealVectorState;

} // namespace statespace
} // namespace aikido

#endif
