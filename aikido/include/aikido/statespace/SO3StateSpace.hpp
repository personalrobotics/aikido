#ifndef AIKIDO_STATESPACE_SO3STATESPACE_H
#define AIKIDO_STATESPACE_SO3STATESPACE_H
#include <Eigen/Geometry>
#include "StateSpace.hpp"
#include "ScopedState.hpp"

namespace aikido
{
namespace statespace
{
template <class T>
class SO3StateHandle;

/// Represents the space of spatial rotations.
class SO3StateSpace : virtual public StateSpace
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
    explicit State(const Quaternion &_quaternion);

    /// Gets value as a transform.
    const Quaternion &getQuaternion() const;

    /// Sets value to a transform.
    void setQuaternion(const Quaternion &_quaternion);

  private:
    Quaternion mValue;

    friend class SO3StateSpace;
  };

  using StateHandle = SO3StateHandle<State>;
  using StateHandleConst = SO3StateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  using Quaternion = State::Quaternion;

  SO3StateSpace() = default;

  ScopedState createState() const;

  /// Gets value as a transform.
  const Quaternion &getQuaternion(const State *_state) const;

  /// Sets value to a transform.
  void setQuaternion(State *_state, const Quaternion &_quaternion) const;

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

  // Documentation inherited. _tangent should be 3d twist.
  void expMap(const Eigen::VectorXd &_tangent,
              StateSpace::State *_out) const override;

  // Documentation inherited
  void logMap(const StateSpace::State *_in,
              Eigen::VectorXd &_tangent) const override;

};

}  // namespace statespace
}  // namespace aikido

#include "detail/SO3StateSpace.hpp"

#endif
