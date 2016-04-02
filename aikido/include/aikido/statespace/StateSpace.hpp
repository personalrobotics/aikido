#ifndef AIKIDO_STATESPACE_STATESPACE_H
#define AIKIDO_STATESPACE_STATESPACE_H

#include "State.hpp"
#include <memory>

namespace aikido {
namespace statespace {

class StateSpace {
public:

  virtual void compose(const State& _state1, const State& _state2,
  									   State& _out) const = 0;

	/// Dimension of represenation of State in this StateSpace.
  virtual int getRepresentationDimension() const = 0;
};

using StateSpacePtr = std::shared_ptr<const StateSpace>;

}
}

#endif
