#ifndef AIKIDO_STATESPACE_SO2STATESPACE_H
#define AIKIDO_STATESPACE_SO2STATESPACE_H
#include "StateSpace.hpp"
#include "State.hpp"
#include "Jacobian.hpp"

namespace aikido {
namespace statespace {

class SO2StateSpace : public StateSpace
{
public:
  /// Point in SO(2), a planar orientation.
  class State : public StateSpace::State
  {
  public:
    /// Constructs the identity element.
    State();

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

  SO2StateSpace() = default;

  // Documentation inherited.
  int getRepresentationDimension() const override;
  
  // Documentation inherited.
  void compose(
    const StateSpace::State& _state1, const StateSpace::State& _state2,
    StateSpace::State& _out) const override;
};

} // namespace statespace
} // namespace aikido

#endif
