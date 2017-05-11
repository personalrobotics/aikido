#ifndef AIKIDO_STATESPACE_SE2STATESPACE_HPP_
#define AIKIDO_STATESPACE_SE2STATESPACE_HPP_
#include <Eigen/Geometry>
#include "ScopedState.hpp"
#include "StateSpace.hpp"

namespace aikido {
namespace statespace {

/// Defined in detail/SE2-impl.hpp
template <class>
class SE2StateHandle;

/// The two-dimensional special Euclidean group SE(2), i.e. the space of planar
/// rigid body transformations. Note that the group operation for SE(2) differs
/// from the group operation of the Cartesian product space R^2 x SO(2) because
/// it is constructed through the semi-direct product.
class SE2 : public virtual StateSpace
{
public:
  class State : public StateSpace::State
  {
  public:
    using Isometry2d
        = Eigen::Transform<double, 2, Eigen::Isometry, Eigen::DontAlign>;

    /// Constructs the identity element.
    State();

    ~State() = default;

    /// Constructs the state from an Eigen transformation object.
    ///
    /// \param _transform Eigen transformation
    explicit State(const Isometry2d& _transform);

    /// Sets value to an Eigen transfomation object.
    ///
    /// \param _transform Eigen transformation
    void setIsometry(const Isometry2d& _transform);

    /// Gets value as an Eigen transformation object.
    ///
    /// \return Eigen trasnformation
    const Isometry2d& getIsometry() const;

  private:
    Isometry2d mTransform;

    friend class SE2;
  };

  using StateHandle = SE2StateHandle<State>;
  using StateHandleConst = SE2StateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  using Isometry2d = State::Isometry2d;

  /// Constructs a state space representing SE(2).
  SE2() = default;

  /// Helper function to create a \c ScopedState.
  ///
  /// \return new \c ScopedState
  ScopedState createState() const;

  /// Gets value as an Eigen transformation object.
  ///
  /// \param _state a \c State in this state space
  /// \return Eigen transformation
  const Isometry2d& getIsometry(const State* _state) const;

  /// Sets value to an Eigen transfomation object.
  ///
  /// \param _state a \c State in this state space
  /// \param _transform Eigen transformation
  void setIsometry(State* _state, const Isometry2d& _transform) const;

  // Documentation inherited.
  size_t getStateSizeInBytes() const override;

  // Documentation inherited.
  StateSpace::State* allocateStateInBuffer(void* _buffer) const override;

  // Documentation inherited.
  void freeStateInBuffer(StateSpace::State* _state) const override;

  // Documentation inherited.
  void compose(
      const StateSpace::State* _state1,
      const StateSpace::State* _state2,
      StateSpace::State* _out) const override;

  // Documentation inherited
  void getIdentity(StateSpace::State* _out) const override;

  // Documentation inherited
  void getInverse(
      const StateSpace::State* _in, StateSpace::State* _out) const override;

  // Documentation inherited
  size_t getDimension() const override;

  // Documentation inherited
  void copyState(
      const StateSpace::State* _source,
      StateSpace::State* _destination) const override;

  /// Exponential mapping of Lie algebra element to a Lie group element. The
  /// tangent space is parameterized a planar twist of the form (rotation,
  /// translation, translation).
  ///
  /// \param _tangent element of the tangent space
  /// \param[out] _out corresponding element of the Lie group
  void expMap(
      const Eigen::VectorXd& _tangent, StateSpace::State* _out) const override;

  /// Log mapping of Lie group element to a Lie algebra element. The tangent
  /// space is parameterized as a planar twist of the form (rotation,
  /// translation, translation).
  ///
  /// \param _state element of this Lie group
  /// \param[out] _tangent corresponding element of the tangent space
  void logMap(const StateSpace::State* _state, Eigen::VectorXd& _tangent)
      const override;

  /// Print the state. Format: [x, y, theta]
  void print(const StateSpace::State* _state, std::ostream& _os) const override;
};

} // namespace statespace
} // namespace aikido

#include "detail/SE2-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_SE2STATESPACE_HPP_
