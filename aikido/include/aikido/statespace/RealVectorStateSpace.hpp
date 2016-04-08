#ifndef AIKIDO_STATESPACE_REALVECTORSTATESPACE_H
#define AIKIDO_STATESPACE_REALVECTORSTATESPACE_H
#include <Eigen/Core>
#include "StateSpace.hpp"
#include "ScopedState.hpp"

#include "../util/RNG.hpp"
#include "../constraint/Sampleable.hpp"

namespace aikido {
namespace statespace {

template <class> class RealVectorStateHandle;

/// Represents a n-dimensional real vector space.
class RealVectorStateSpace
  : public std::enable_shared_from_this<RealVectorStateSpace>
  , public virtual StateSpace
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

  using Bounds = Eigen::Matrix<double, Eigen::Dynamic, 2>;

  /// Constructs an unbounded RealVectorStateSpace.
  explicit RealVectorStateSpace(int _dimension);

  /// Constructs a RealVectorStateSpace with the given bounds.
  explicit RealVectorStateSpace(const Bounds& _bounds);

  /// Helper function to create a ScopedState.
  ScopedState createState() const;

  /// Gets the dimension of this space.
  int getDimension() const;

  /// Gets the upper and lower bounds of this space.
  const Bounds& getBounds() const;

  /// Generate samples uniformly at random from this state space.
  constraint::SampleableConstraintPtr createSampleableConstraint(
    std::unique_ptr<util::RNG> _rng) const;

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
  /// Gets the value stored in a RealVectorStateSpace::State.
  Eigen::Map<Eigen::VectorXd> getMutableValue(State* _state) const;

  Bounds mBounds;
};

} // namespace statespace
} // namespace aikido

#include "detail/RealVectorStateSpace.hpp"

#endif
