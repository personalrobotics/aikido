#ifndef AIKIDO_STATESPACE_REALVECTORSTATESPACE_HPP_
#define AIKIDO_STATESPACE_REALVECTORSTATESPACE_HPP_
#include <Eigen/Core>
#include "StateSpace.hpp"
#include "ScopedState.hpp"

namespace aikido {
namespace statespace {

// Defined in detail/Rn-impl.hpp
template <class>
class RealVectorStateHandle;

/// Represents a n-dimensional real vector space with vector addition as the
/// group operation.
template <int N>
class Rn : public virtual StateSpace
{
public:
  /// Point in a \c Rn.
  class State : public StateSpace::State
  {
  public:
    static constexpr int Dimension = N;

  protected:
    State() = default;
    ~State() = default;

    friend class Rn<N>;
  };

  /// Dimension of the space
  static constexpr int Dimension = N;

  using VectorNd = Eigen::Matrix<double, N, 1>;

  using StateHandle = RealVectorStateHandle<State>;
  using StateHandleConst = RealVectorStateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  /// Constructs a \c _dimension dimensional real vector space.
  Rn();

  /// Helper function to create a \c ScopedState.
  ///
  /// \return new \c ScopedState
  ScopedState createState() const;

  /// Gets the real vector stored in a \c State.
  ///
  /// \param _state a \c State in this state space
  /// \return real vector represented by \c _state
  Eigen::Map<const VectorNd> getValue(const State *_state) const;

  /// Sets the real vector stored in a \c State.
  ///
  /// \param _state a \c State in this state space
  /// \param _value real vector to store in \c _state
  void setValue(State *_state, const VectorNd &_value) const;

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
  size_t getDimension() const override;

  // Documentation inherited
  void copyState(
    const StateSpace::State *_source,
    StateSpace::State *_destination) const override;

  /// Exponential mapping of Lie algebra element to a Lie group element. This
  /// is simply the identity transformation on a real vector space.
  ///
  /// \param _tangent element of the tangent space
  /// \param[out] _out corresponding element of the Lie group
  void expMap(const Eigen::VectorXd &_tangent,
              StateSpace::State *_out) const override;

  /// Log mapping of Lie group element to a Lie algebra element. This is simply
  /// an identity transformation on a real vector space.
  ///
  /// \param _state element of this Lie group
  /// \param[out] _tangent corresponding element of the tangent space
  void logMap(const StateSpace::State *_in,
              Eigen::VectorXd &_tangent) const override;

  /// Print the n-dimensional vector represented by the state
  /// Format: [x_1, x_2, ..., x_n]
  void print(const StateSpace::State *_state, std::ostream &_os) const override;

private:
  /// Gets the mutable value stored in a Rn::State. This is
  /// used internally to implement the public \c getValue member functions.
  ///
  /// \param _state element of this state space
  /// \return mutable reference to real vector stored in \c _state
  Eigen::Map<VectorNd> getMutableValue(State *_state) const;

  int mDimension;
};

using R0 = Rn<0>;
using R1 = Rn<1>;
using R2 = Rn<2>;
using R3 = Rn<3>;
using R6 = Rn<6>;

} // namespace statespace
} // namespace aikido

#include "detail/Rn-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_REALVECTORSTATESPACE_HPP_
