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
  class State : public StateSpace::State
  {
  public:
    /// Constructs the identity element.
    State();

    ~State() = default;

    /// Constructs a state from a rotation angle.
    ///
    /// \param _angle rotation angle
    explicit State(double _angle);

    /// Gets state as a rotation angle.
    ///
    /// \return rotation angle
    double getAngle() const;

    /// Sets state to a rotation angle.
    ///
    /// \param _angle rotation angle
    void setAngle(double _angle);

    /// Gets state as an Eigen transformation.
    ///
    /// \return Eigen transformation
    Eigen::Rotation2Dd getRotation() const;

    /// Sets state it an Eigen transformation.
    ///
    /// \param _rotation Eigen transformation
    void setRotation(const Eigen::Rotation2Dd& _rotation);

  private:
    double mAngle;

    friend class SO2;
  };

  using StateHandle = SO2StateHandle<State>;
  using StateHandleConst = SO2StateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  /// Constructs a state space representing SO(2).
  SO2() = default;

  /// Helper function to create a \c ScopedState.
  ///
  /// \return new \c ScopedState
  ScopedState createState() const;

  /// Gets state as a rotation angle.
  ///
  /// \param _state input state
  /// \return rotation angle
  double getAngle(const State* _state) const;

  /// Sets state to a rotation angle.
  ///
  /// \param _state input state
  /// \param _angle rotation angle
  void setAngle(State* _state, double _angle) const;

  /// Gets state as an Eigen transformation.
  ///
  /// \param _state input state
  /// \return Eigen transformation
  Eigen::Rotation2Dd getRotation(const State* _state) const;

  /// Sets state it an Eigen transformation.
  ///
  /// \param _state input state
  /// \param _rotation Eigen transformation
  void setRotation(State* _state, const Eigen::Rotation2Dd& _rotation) const;

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

  // Documentation inherited.
  void getIdentity(StateSpace::State* _out) const override;

  // Documentation inherited.
  void getInverse(
      const StateSpace::State* _in, StateSpace::State* _out) const override;

  // Documentation inherited.
  size_t getDimension() const override;

  // Documentation inherited.
  void copyState(
      const StateSpace::State* _source,
      StateSpace::State* _destination) const override;

  /// Exponential mapping of Lie algebra element to a Lie group element. The
  /// tangent space is parameterized a rotation angle.
  ///
  /// \param _tangent element of the tangent space
  /// \param[out] _out corresponding element of the Lie group
  void expMap(
      const Eigen::VectorXd& _tangent, StateSpace::State* _out) const override;

  /// Log mapping of Lie group element to a Lie algebra element. The tangent
  /// space is parameterized as a rotation angle.
  ///
  /// \param _state element of this Lie group
  /// \param[out] _tangent corresponding element of the tangent space
  void logMap(
      const StateSpace::State* _in, Eigen::VectorXd& _tangent) const override;

  /// Print the angle represented by the state
  void print(const StateSpace::State* _state, std::ostream& _os) const override;
};

} // namespace statespace
} // namespace aikido

#include "detail/SO2-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_SO2STATESPACE_HPP_
