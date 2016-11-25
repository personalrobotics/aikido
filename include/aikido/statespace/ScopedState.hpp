#ifndef AIKIDO_STATESPACE_SCOPEDSTATE_HPP_
#define AIKIDO_STATESPACE_SCOPEDSTATE_HPP_
#include <memory>
#include "StateHandle.hpp"

namespace aikido {
namespace statespace {

/// CRTP RAII wrapper for a \c StateHandle. The constructor of \c ScopedState
/// allocates a state and the destructor destroys it.
///
/// \tparam _Handle \c StateHandle class being wrapped.
template <class _Handle>
class ScopedState : public _Handle
{
public:
  using Handle = _Handle;
  using typename Handle::StateSpace;
  using typename Handle::State;
  using typename Handle::QualifiedState;

  /// Construct a \c ScopedState by allocating a new state in \c _space. This
  /// state will be freed when \c ScopedState is destructed.
  ///
  /// \param _space state 
  explicit ScopedState(const StateSpace* _space);

  virtual ~ScopedState();

  // ScopedState is uncopyable, must use std::move
  ScopedState(const ScopedState&) = delete;
  ScopedState& operator =(const ScopedState&) = delete;

  ScopedState(ScopedState&&) = default;
  ScopedState& operator =(ScopedState&&) = default;

private:
  std::unique_ptr<char[]> mBuffer;
};

} // namespace statespace
} // namespace aikido

#include "detail/ScopedState-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_SCOPEDSTATE_HPP_
