#ifndef AIKIDO_STATESPACE_SO3STATESPACE_H
#define AIKIDO_STATESPACE_SO3STATESPACE_H
#include "StateSpace.hpp"
#include "State.hpp"
#include "Jacobian.hpp"

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
    /// Constructs the identity element.
    State();

    /// Constructs a point in SO(3) from a quaternion.
    explicit State(const Eigen::Quaterniond& _quaternion);

    /// Gets value as a transform.
    const Eigen::Quaterniond& getQuaternion() const;

    /// Sets value to a transform.
    void setQuaternion(const Eigen::Quaterniond& _quaternion);

    // Required because Quaterniond is a fixed-size vectorizable type.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    Eigen::Quaterniond mValue;

    friend class SO3StateSpace;
  };

  SO3StateSpace() = default;

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
