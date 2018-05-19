#ifndef AIKIDO_STATESPACE_SO2STATESPACE_HPP_
#define AIKIDO_STATESPACE_SO2STATESPACE_HPP_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ScopedState.hpp"
#include "StateSpace.hpp"

namespace aikido {
namespace statespace {

// Defined in detail/SO2-impl.hpp
template <class>
class SO2StateHandle;

/// The two-dimensional special orthogonal group SO(2), i.e. the space of
/// planar rigid body rotations.
class SO2 : virtual public StateSpace
{
public:
  /// State in SO(2), a planar rotation.
  class State;

  using StateHandle = SO2StateHandle<State>;
  using StateHandleConst = SO2StateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  using StateSpace::compose;

  /// Constructs a state space representing SO(2).
  SO2() = default;

  /// Helper function to create a \c ScopedState.
  ///
  /// \return new \c ScopedState.
  ScopedState createState() const;

  /// Creates an identical clone of \c stateIn.
  ScopedState cloneState(const StateSpace::State* stateIn) const;

  /// Returns state as a rotation angle in (-pi, pi].
  ///
  /// \param[in] state State.
  double toAngle(const State* state) const;

  /// Sets state from a rotation angle in (-inf, inf).
  ///
  /// \param[out] state State corresponding to angle.
  /// \param[in] angle Rotation angle in (-inf, inf).
  void fromAngle(State* state, double angle) const;

  /// Returns state as an Eigen rotation.
  ///
  /// \param[in] state State.
  Eigen::Rotation2Dd toRotation(const State* state) const;

  /// Sets state from an Eigen rotation.
  ///
  /// \param[out] state State corresponding to rotation.
  /// \param[in] rotation Eigen rotation.
  void fromRotation(State* state, const Eigen::Rotation2Dd& rotation) const;

  // Documentation inherited.
  std::size_t getStateSizeInBytes() const override;

  // Documentation inherited.
  StateSpace::State* allocateStateInBuffer(void* buffer) const override;

  // Documentation inherited.
  void freeStateInBuffer(StateSpace::State* state) const override;

  // Documentation inherited.
  void compose(
      const StateSpace::State* state1,
      const StateSpace::State* state2,
      StateSpace::State* out) const override;

  // Documentation inherited.
  void getIdentity(StateSpace::State* out) const override;

  // Documentation inherited.
  void getInverse(
      const StateSpace::State* in, StateSpace::State* out) const override;

  // Documentation inherited.
  std::size_t getDimension() const override;

  // Documentation inherited.
  void copyState(
      const StateSpace::State* source,
      StateSpace::State* destination) const override;

  /// Exponential mapping of Lie algebra element to a Lie group element. The
  /// tangent space is parameterized a rotation angle.
  ///
  /// \param[in] tangent Element of the tangent space.
  /// \param[out] out Corresponding element of the Lie group.
  void expMap(
      const Eigen::VectorXd& tangent, StateSpace::State* out) const override;

  /// Log mapping of Lie group element to a Lie algebra element. The tangent
  /// space is parameterized as a rotation angle.
  ///
  /// \param[in] in Element of this Lie group.
  /// \param[out] tangent Corresponding element of the tangent space.
  void logMap(
      const StateSpace::State* in, Eigen::VectorXd& tangent) const override;

  /// Print the angle represented by the state.
  void print(const StateSpace::State* state, std::ostream& os) const override;
};

class SO2::State final : public StateSpace::State
{
public:
  /// Constructs a state from a rotation angle.
  ///
  /// \param[in] angle Rotation angle in (-inf, inf).
  explicit State(double angle = 0.0);

  ~State() = default;

  /// Returns state as a rotation angle in (-pi, pi].
  double toAngle() const;

  /// Sets state from a rotation angle.
  ///
  /// \param[in] angle Rotation angle in (-inf, inf).
  void fromAngle(double angle);

  /// Returns state as an Eigen rotation.
  Eigen::Rotation2Dd toRotation() const;

  /// Sets state from an Eigen rotation.
  ///
  /// \param[in] rotation Eigen rotation.
  void fromRotation(const Eigen::Rotation2Dd& rotation);

private:
  /// Angle bounded in (-pi, pi] representing the SO(2) state.
  double mAngle;

  friend class SO2;
};

} // namespace statespace
} // namespace aikido

#include "detail/SO2-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_SO2STATESPACE_HPP_
