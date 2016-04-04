#ifndef AIKIDO_STATESPACE_SO3STATESPACE_H
#define AIKIDO_STATESPACE_SO3STATESPACE_H
#include <Eigen/Geometry>
#include "StateSpace.hpp"

namespace aikido {
namespace statespace {

/// Represents the space of spatial rotations.
class SO3StateSpace : public StateSpace
{
public:
  /// Point in SO(3), a spatial orientation.
  class State : public StateSpace::State
  {
  public:
    using Quaternion = Eigen::Quaternion<double, Eigen::DontAlign>;

    /// Constructs the identity element.
    State();

    ~State() = default;

    /// Constructs a point in SO(3) from a quaternion.
    explicit State(const Quaternion& _quaternion);

    /// Gets value as a transform.
    const Quaternion& getQuaternion() const;

    /// Sets value to a transform.
    void setQuaternion(const Quaternion& _quaternion);

  private:
    Quaternion mValue;

    friend class SO3StateSpace;
  };

  SO3StateSpace() = default;

  // Documentation inherited.
  size_t getStateSizeInBytes() const override;

  // Documentation inherited.
  StateSpace::State* allocateStateInBuffer(void* _buffer) const;

  // Documentation inherited.
  void freeStateInBuffer(StateSpace::State* _state) const override;
  
  // Documentation inherited.
  void compose(
    const StateSpace::State& _state1, const StateSpace::State& _state2,
    StateSpace::State& _out) const override;
};

} // namespace statespace
} // namespace aikido

#endif
