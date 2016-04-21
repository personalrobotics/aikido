#ifndef AIKIDO_STATESPACE_JOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_JOINTSTATESPACE_HPP_
#include <dart/dynamics/dynamics.h>
#include "../StateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c StateSpace of a DART \c Joint. This is a base class that is inherited by
/// concrete implementations for DART's various \c Joint subclasses. This class
/// provides functions for converting between \c State objects and vectors of
/// DART joint positions.
class JointStateSpace : public virtual StateSpace
{
public:
  /// Constructs a state space for \c _joint.
  ///
  /// \param _joint joint to create a \c StateSpace for 
  explicit JointStateSpace(::dart::dynamics::Joint* _joint);

  virtual ~JointStateSpace() = default;

  /// Gets the joint associated with this state space.
  ///
  /// \return joint associated with this state space
  ::dart::dynamics::Joint* getJoint() const;

  /// Converts DART \c Joint positions, e.g. those returned by
  /// \c getPositions, to a \c State in this state space.
  ///
  /// \param _positions input DART \c Joint positions
  /// \param[out] _state output state
  virtual void convertPositionsToState(
    const Eigen::VectorXd& _positions, StateSpace::State* _state) const = 0;

  /// Converts a \c State in this state space to DART \c Joint positions, e.g.
  /// that may be passed to \c setPositions.
  ///
  /// \param _state input state 
  /// \param[out] _positions output DART \c Joint positions
  virtual void convertStateToPositions(
    const StateSpace::State* _state, Eigen::VectorXd& _positions) const = 0;

  /// Gets the positions of the \c Joint and store them in \c _state.
  ///
  /// \param[out] _state output state
  virtual void getState(StateSpace::State* _state) const;

  /// Sets the positions of the \c Joint to \c _state.
  ///
  /// \param _state input state
  virtual void setState(const StateSpace::State* _state) const;

protected:
  ::dart::dynamics::Joint* mJoint;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_JOINTSTATESPACE_HPP_
