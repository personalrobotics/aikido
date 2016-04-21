#ifndef AIKIDO_STATESPACE_DART_METASKELETONSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_METASKELETONSTATESPACE_HPP_
#include <dart/dynamics/dynamics.h>
#include "../CompoundStateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c StateSpace of a DART \c MetaSkeleton. This is a \c CompoundStateSpace,
/// where the i-th subspace is a \c JointStateSpace for the i-th \c Joint of
/// the \c MetaSkeleton. This class provides functions for converting between
/// \c State objects and vectors of DART joint positions.
///
/// The behavior of this class is undefined if you modify the structure of the
/// \c MetaSkeleton or its position limits after construction.
class MetaSkeletonStateSpace : public CompoundStateSpace
{
public:
  using CompoundStateSpace::State;
  using CompoundStateSpace::ScopedState;

  /// Constructs a state space for a DART \c MetaSkeleton.
  ///
  /// \param _metaskeleton target \c MetaSkeleton
  MetaSkeletonStateSpace(::dart::dynamics::MetaSkeletonPtr _metaskeleton);

  /// Get the \c MetaSkeleton associated with this state space.
  ///
  /// \return \c MetaSkeleton associated with this state space
  ::dart::dynamics::MetaSkeletonPtr getMetaSkeleton() const;

  /// Gets the subspace corresponding to \c _joint.
  ///
  /// \tparam Space type of \c StateSpace to return
  /// \param _joint joint in this \c MetaSkeleton
  /// \return state space corresponding to \c _joint
  template <class Space = JointStateSpace>
  std::shared_ptr<Space> getJointSpace(
    const ::dart::dynamics::Joint* _joint) const;

  /// Gets the subspace corresponding to joint with index \c _index.
  ///
  /// \tparam Space type of \c StateSpace to return
  /// \param _index index of a joint in the \c MetaSkeleton
  /// \return state space corresponding to \c _joint
  template <class Space = JointStateSpace>
  std::shared_ptr<Space> getJointSpace(size_t _index) const;

  /// Converts DART \c MetaSkeleton positions, e.g. those returned by
  /// \c getPositions, to a \c State in this state space.
  ///
  /// \param _positions input DART \c MetaSkeleton positions
  /// \param[out] _state output state
  void convertPositionsToState(
    const Eigen::VectorXd& _positions, State* _state) const;

  /// Converts a \c State in this state space to DART \c MetaSkeleton
  /// positions, e.g. that may be passed to \c setPositions.
  ///
  /// \param _state input state 
  /// \param[out] _positions output DART \c MetaSkeleton positions
  void convertStateToPositions(
    const State* _state, Eigen::VectorXd& _positions) const;

  /// Gets the positions of the \c MetaSkeleton and store them in \c _state.
  ///
  /// \param[out] _state output state
  void getState(State* _state) const;

  /// Sets the positions of the \c MetaSkeleton to \c _state.
  ///
  /// \param _state input state
  void setState(const State* _state);

  /// Wrapper for \c getStateFromMetaSkeleton that returns a ScopedState.
  ///
  /// \return current state of the \c MetaSkeleton
  ScopedState getScopedStateFromMetaSkeleton() const;

private:
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

using MetaSkeletonStateSpacePtr = std::shared_ptr<MetaSkeletonStateSpace>;

/// Create a JointStateSpace for a Joint whose type is known at compile time.
///
/// \tparam JointType joint type
/// \param _joint joint to create a state space for
/// \return state space of \c _joint
template <class JointType>
std::unique_ptr<JointStateSpace> createJointStateSpaceFor(JointType* _joint);

/// Create a JointStateSpace for an aribtrary Joint. This may be significantly
/// slower than createJointStateSpaceFor(), so only use this function if the
/// Joint's type is not known at compile time.
///
/// \param _joint joint to create a state space for
/// \return state space of \c _joint
std::unique_ptr<JointStateSpace> createJointStateSpace(
  ::dart::dynamics::Joint* _joint);

} // namespace dart
} // namespace statespace
} // namespace aikido

#include "detail/MetaSkeletonStateSpace-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_DART_METASKELETONSTATESPACE_HPP_
