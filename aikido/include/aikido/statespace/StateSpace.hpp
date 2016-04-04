#ifndef AIKIDO_STATESPACE_STATESPACE_H
#define AIKIDO_STATESPACE_STATESPACE_H
#include <memory>

namespace aikido {
namespace statespace {

/// Base class for all StateSpaces.
class StateSpace {
public:
  /// Base class for all States.
  class State
  {
  public:
    // TODO: Remove this virtual destructor. Having any virtual function in
    // this class, including this destructor, adds sizeof(pointer) overhead.
    virtual ~State() = default;

  protected:
    State() = default;
  };

  virtual ~StateSpace() = default;

  /// TODO: Need a docstring for this.
  virtual void compose(const State& _state1, const State& _state2,
                       State& _out) const = 0;
};

using StateSpacePtr = std::shared_ptr<StateSpace>;

} // namespace statespace
} // namespace aikido

#endif
