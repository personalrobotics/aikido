#ifndef AIKIDO_STATESPACE_SO3STATESPACE_HPP_
#define AIKIDO_STATESPACE_SO3STATESPACE_HPP_
#include <Eigen/Geometry>
#include "ScopedState.hpp"
#include "StateSpace.hpp"

namespace aikido {
namespace statespace {

// Defined in detail/SO3-impl.hpp
template <class>
class SO3StateHandle;

/// The two-dimensional special orthogonal group SO(3), i.e. the space of
/// spatial rigid body rotations.
class SO3 : virtual public StateSpace
{
public:
  /// State in SO(3), a spatial rotation.
  class State : public StateSpace::State
  {
  public:
    using Quaternion = Eigen::Quaternion<double, Eigen::DontAlign>;

    /// Constructs the identity element.
    State();

    ~State() = default;

    /// Constructs a state in SO(3) from a unit quaternion.
    ///
    /// \param _quaternion unit quaternion representing orientation
    explicit State(const Quaternion& _quaternion);

    /// Gets a state as a unit quaternion.
    ///
    /// \return unit quaternion representing orientation
    const Quaternion& getQuaternion() const;

    /// Sets a state to a unit quaternion.
    ///
    /// \param _quaternion unit quaternion representing orientation
    void setQuaternion(const Quaternion& _quaternion);

  private:
    Quaternion mValue;

    friend class SO3;
  };

  using StateHandle = SO3StateHandle<State>;
  using StateHandleConst = SO3StateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  using Quaternion = State::Quaternion;

  /// Constructs a state space representing SO(3).
  SO3() = default;

  /// Helper function to create a \c ScopedState.
  ///
  /// \return new \c ScopedState
  ScopedState createState() const;

  /// Gets a state as a unit quaternion.
  ///
  /// \param _state input state
  /// \return unit quaternion representing orientation
  const Quaternion& getQuaternion(const State* _state) const;

  /// Sets a state to a unit quaternion.
  ///
  /// \param _state input state
  /// \param _quaternion unit quaternion representing orientation
  void setQuaternion(State* _state, const Quaternion& _quaternion) const;

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
  /// tangent space is parameterized as a spatial rotation velocity.
  ///
  /// \param _tangent element of the tangent space
  /// \param[out] _out corresponding element of the Lie group
  void expMap(
      const Eigen::VectorXd& _tangent, StateSpace::State* _out) const override;

  /// Log mapping of Lie group element to a Lie algebra element. The tangent
  /// space is parameterized as a spatial rotational velocity.
  ///
  /// \param _state element of this Lie group
  /// \param[out] _tangent corresponding element of the tangent space
  void logMap(
      const StateSpace::State* _in, Eigen::VectorXd& _tangent) const override;

  /// Print the quaternion represented by the state.
  /// Format: [w, x, y, z]
  void print(const StateSpace::State* _state, std::ostream& _os) const override;
};

} // namespace statespace
} // namespace aikido

#include "detail/SO3-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_SO3STATESPACE_HPP_
