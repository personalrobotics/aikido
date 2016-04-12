#ifndef AIKIDO_STATESPACE_SO2STATESPACE_H
#define AIKIDO_STATESPACE_SO2STATESPACE_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "StateSpace.hpp"
#include "ScopedState.hpp"

namespace aikido
{
namespace statespace
{
template <class>
class SO2StateHandle;

/// Represents the space of planar rotations.
class SO2StateSpace : virtual public StateSpace
{
public:
  /// Point in SO(2), a planar rotation.
  class State : public StateSpace::State
  {
  public:
    /// Constructs the identity element.
    State();

    ~State() = default;

    /// Constructs a point in SO(2) from an angle in radians.
    explicit State(double _angle);

    /// Gets the angle of the rotation encoded by this state.
    double getAngle() const;

    /// Sets this state to a rotation by the specified angle.
    void setAngle(double _angle);

    /// Gets value as a rigid body rotation.
    Eigen::Rotation2Dd getRotation() const;

    /// Sets this state to the given rotation.
    void setRotation(const Eigen::Rotation2Dd& _rotation);

  private:
    double mAngle;

    friend class SO2StateSpace;
  };

  using StateHandle = SO2StateHandle<State>;
  using StateHandleConst = SO2StateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  SO2StateSpace() = default;

  ScopedState createState() const;

  /// Gets the angle of the rotation encoded by this state.
  double getAngle(const State* _state) const;

  /// Sets this state to a rotation by the specified angle.
  void setAngle(State* _state, double _angle) const;

  /// Gets value as a rigid body rotation.
  Eigen::Rotation2Dd getRotation(const State* _state) const;

  /// Sets this state to the given rotation.
  void setRotation(State* _state, const Eigen::Rotation2Dd& _rotation) const;

  // Documentation inherited.
  size_t getStateSizeInBytes() const override;

  // Documentation inherited.
  StateSpace::State* allocateStateInBuffer(void* _buffer) const override;

  // Documentation inherited.
  void freeStateInBuffer(StateSpace::State* _state) const override;

  // Documentation inherited.
  void compose(const StateSpace::State* _state1,
               const StateSpace::State* _state2,
               StateSpace::State* _out) const override;

  // Documentation inherited
  void getIdentity(StateSpace::State* _out) const override;

  // Documentation inherited
  void getInverse(const StateSpace::State *_in,
                  StateSpace::State *_out) const override;

  // Documentation inherited
  unsigned int getDimension() const override;

  // Documentation inherited
  double getMaximumExtent() const override;

  // Documentation inherited
  double getMeasure() const override;

  // Documentation inherited
  void copyState(StateSpace::State* _destination,
                 const StateSpace::State* _source) const override;

  // Documentation inherited
  double distance(const StateSpace::State* _state1,
                  const StateSpace::State* _state2) const override;

  // Documentation inherited
  bool equalStates(const StateSpace::State* _state1,
                   const StateSpace::State* _state2) const override;

  // Documentation inherited
  void interpolate(const StateSpace::State* _from, const StateSpace::State* _to,
                   const double _t, StateSpace::State* _State) const override;

  // Documentation inherited.
  void expMap(const Eigen::VectorXd& _tangent,
              StateSpace::State* _out) const override;
};

}  // namespace statespace
}  // namespace aikido

#include "detail/SO2StateSpace.hpp"

#endif
