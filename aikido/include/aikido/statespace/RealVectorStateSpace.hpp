#ifndef AIKIDO_STATESPACE_REALVECTORSTATESPACE_HPP_
#define AIKIDO_STATESPACE_REALVECTORSTATESPACE_HPP_
#include <Eigen/Core>
#include "StateSpace.hpp"
#include "ScopedState.hpp"

namespace aikido {
namespace statespace {

/// Defined in detail/RealVectorStateSpace-impl.hpp
///
template <class>
class RealVectorStateHandle;

/// Represents a n-dimensional real vector space.
class RealVectorStateSpace : public virtual StateSpace
{
public:
  /// Point in a \c RealVectorStateSpace.
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

  /// Constructs a \c _dimension dimensional real vector space.
  ///
  /// \param _dimension dimension of the space
  explicit RealVectorStateSpace(int _dimension);

  /// Helper function to create a \c ScopedState.
  ///
  /// \return new \c ScopedState
  ScopedState createState() const;

  /// Gets the real vector stored in a \c State.
  ///
  /// \param _state a \c State in this state space
  /// \return real vector represented by \c _state
  Eigen::Map<const Eigen::VectorXd> getValue(const State *_state) const;

  /// Sets the real vector stored in a \c State.
  ///
  /// \param _state a \c State in this state space
  /// \param _value real vector to store in \c _state
  void setValue(State *_state, const Eigen::VectorXd &_value) const;

  // Documentation inherited.
  size_t getStateSizeInBytes() const override;

  // Documentation inherited.
  StateSpace::State *allocateStateInBuffer(void *_buffer) const override;

  // Documentation inherited.
  void freeStateInBuffer(StateSpace::State *_state) const override;

  // Documentation inherited.
  void compose(const StateSpace::State *_state1,
               const StateSpace::State *_state2,
               StateSpace::State *_out) const override;

  // Documentation inherited
  void getIdentity(StateSpace::State *_out) const override;

  // Documentation inherited
  void getInverse(const StateSpace::State *_in,
                  StateSpace::State *_out) const override;

  // Documentation inherited
  unsigned int getDimension() const override;

  // Documentation inherited
  void copyState(StateSpace::State* _destination,
                 const StateSpace::State* _source) const override;

  // Documentation inherited.
  void expMap(const Eigen::VectorXd &_tangent,
              StateSpace::State *_out) const override;

  // Documentation inherited
  void logMap(const StateSpace::State *_in,
              Eigen::VectorXd &_tangent) const override;

private:
  /// Gets the value stored in a RealVectorStateSpace::State.
  Eigen::Map<Eigen::VectorXd> getMutableValue(State *_state) const;

  int mDimension;
};

} // namespace statespace
} // namespace aikido

#include "detail/RealVectorStateSpace-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_REALVECTORSTATESPACE_HPP_
