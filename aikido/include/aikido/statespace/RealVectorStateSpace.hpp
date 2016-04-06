#ifndef AIKIDO_STATESPACE_REALVECTORSTATESPACE_H
#define AIKIDO_STATESPACE_REALVECTORSTATESPACE_H
#include <Eigen/Core>
#include "StateSpace.hpp"
#include "ScopedState.hpp"

namespace aikido {
namespace statespace {

// Defined in detail/RealVectorStateSpace.hpp
template <class> class RealVectorStateHandle;

/// Represents a n-dimensional real vector space.
class RealVectorStateSpace : public StateSpace
{
public:
  /// Point in a RealVectorStateSpace.
  class State : public StateSpace::State
  {
  protected:
    State() = default;
    ~State() = default;

    friend class RealVectorStateSpace;
  };

  using StateHandle = RealVectorStateHandle<State>;
  using StateHandleConst = RealVectorStateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  /// Constructs a RealVectorStateSpace with the given dimensionality.
  explicit RealVectorStateSpace(int _dimension);

  /// Helper function to create a ScopedState.
  ScopedState createState() const;

  /// Gets the value stored in a RealVectorStateSpace::State.
  Eigen::Map<Eigen::VectorXd> getValue(State* _state) const;

  /// Gets the value stored in a RealVectorStateSpace::State.
  Eigen::Map<const Eigen::VectorXd> getValue(const State* _state) const;

  /// Gets the value stored in a RealVectorStateSpace::State.
  void setValue(State* _state, const Eigen::VectorXd& _value) const;

  // Documentation inherited.
  size_t getStateSizeInBytes() const override;

  // Documentation inherited.
  StateSpace::State* allocateStateInBuffer(void* _buffer) const override;

  // Documentation inherited.
  void freeStateInBuffer(StateSpace::State* _state) const override;

  // Documentation inherited.
  void compose(
    const StateSpace::State* _state1, const StateSpace::State* _state2,
    StateSpace::State* _out) const override;

private:
  int mDimension;
};

} // namespace statespace
} // namespace aikido

#include "detail/RealVectorStateSpace.hpp"

#endif
