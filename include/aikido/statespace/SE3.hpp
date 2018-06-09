#ifndef AIKIDO_STATESPACE_SE3STATESPACE_HPP_
#define AIKIDO_STATESPACE_SE3STATESPACE_HPP_
#include <Eigen/Geometry>
#include "ScopedState.hpp"
#include "StateSpace.hpp"

namespace aikido {
namespace statespace {

// Defined in detail/SE3-impl.hpp
template <class>
class SE3StateHandle;

/// The three-dimensional special Euclidean group SE(3), i.e. the space of
/// spatial rigid body transformations. Note that the group operation for SE(3)
/// differs from the group operation of the Cartesian product space R^3 x SO(3)
/// because it is constructed through the semi-direct product.
class SE3 : public virtual StateSpace
{
public:
  class State : public StateSpace::State
  {
  public:
    using Isometry3d
        = Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign>;

    /// Constructs the identity element.
    State();

    ~State() = default;

    /// Constructs the state from an Eigen transformation object.
    ///
    /// \param _transform Eigen transformation
    explicit State(const Isometry3d& _transform);

    /// Sets value to an Eigen transfomation object.
    ///
    /// \param _transform Eigen transformation
    void setIsometry(const Isometry3d& _transform);

    /// Gets value as an Eigen transformation object.
    ///
    /// \return Eigen trasnformation
    const Isometry3d& getIsometry() const;

  private:
    Isometry3d mTransform;

    friend class SE3;
  };

  using StateHandle = SE3StateHandle<State>;
  using StateHandleConst = SE3StateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  using StateSpace::compose;

  using Isometry3d = State::Isometry3d;

  /// Constructs a state space representing SE(3).
  SE3() = default;

  /// Helper function to create a \c ScopedState.
  ///
  /// \return new \c ScopedState
  ScopedState createState() const;

  /// Creates an identical clone of \c stateIn.
  ScopedState cloneState(StateSpace::State* stateIn) const;

  /// Gets value as an Eigen transformation object.
  ///
  /// \param _state a \c State in this state space
  /// \return Eigen transformation
  const Isometry3d& getIsometry(const State* _state) const;

  /// Sets value to an Eigen transfomation object.
  ///
  /// \param _state a \c State in this state space
  /// \param _transform Eigen transformation
  void setIsometry(State* _state, const Isometry3d& _transform) const;

  // Documentation inherited.
  std::size_t getStateSizeInBytes() const override;

  // Documentation inherited.
  StateSpace::State* allocateStateInBuffer(void* _buffer) const override;

  // Documentation inherited.
  void freeStateInBuffer(const StateSpace::State* _state) const override;

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
  std::size_t getDimension() const override;

  // Documentation inherited.
  void copyState(
      const StateSpace::State* _source,
      StateSpace::State* _destination) const override;

  /// Exponential mapping of Lie algebra element to a Lie group element. The
  /// tangent space is parameterized a planar twist of the form (rotation,
  /// translation).
  ///
  /// \param _tangent element of the tangent space
  /// \param[out] _out corresponding element of the Lie group
  void expMap(
      const Eigen::VectorXd& _tangent, StateSpace::State* _out) const override;

  /// Log mapping of Lie group element to a Lie algebra element. The tangent
  /// space is parameterized as a planar twist of the form (rotation,
  /// translation).
  ///
  /// \param _in element of this Lie group
  /// \param[out] _tangent corresponding element of the tangent space
  void logMap(
      const StateSpace::State* _in, Eigen::VectorXd& _tangent) const override;

  /// Print the quaternion followed by the translation
  /// Format: [q.w, q.x, q.y, q.z, x, y, z] where is the quaternion
  /// representation of the rotational component of the state
  void print(const StateSpace::State* _state, std::ostream& _os) const override;
};

} // namespace statespace
} // namespace aikido

#include "detail/SE3-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_SE3STATESPACE_HPP_
