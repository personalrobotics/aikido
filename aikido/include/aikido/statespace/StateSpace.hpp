#ifndef AIKIDO_STATESPACE_STATESPACE_H
#define AIKIDO_STATESPACE_STATESPACE_H
#include "State.hpp"
#include "Jacobian.hpp"
#include <memory>

namespace aikido {
namespace statespace {

class StateSpace {
public:
  typedef ::aikido::statespace::State State;
  typedef ::aikido::statespace::Jacobian Jacobian;

  virtual ~StateSpace() = default;

  virtual void compose(const State& _state1, const State& _state2,
                       State& _out) const = 0;

  /// Dimension of represenation of State in this StateSpace.
  virtual int getRepresentationDimension() const = 0;
};

using StateSpacePtr = std::shared_ptr<const StateSpace>;

} // namespace statespace
} // namespace aikido

#endif
