#ifndef AIKIDO_STATESPACE_METASKELETONSTATESPACE_HPP_
#define AIKIDO_STATESPACE_METASKELETONSTATESPACE_HPP_
#include <dart/dynamics/dynamics.h>
#include "../CompoundStateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// StateSpace that represents the configuration space of a MetaSkeleton.
class MetaSkeletonStateSpace : public CompoundStateSpace
{
public:
  using CompoundStateSpace::State;
  using CompoundStateSpace::ScopedState;

  /// Create a StateSpace for the Joints in the MetaSkeleton.
  MetaSkeletonStateSpace(::dart::dynamics::MetaSkeletonPtr _metaskeleton);

  /// Get the MetaSkeleton associated with this StateSpace.
  ::dart::dynamics::MetaSkeletonPtr getMetaSkeleton() const;

  template <class Space = JointStateSpace>
  std::shared_ptr<Space> getJointSpace(
    const ::dart::dynamics::Joint* _joint) const;

  template <class Space = JointStateSpace>
  std::shared_ptr<Space> getJointSpace(size_t _index) const;

  void convertPositionsToState(
    const Eigen::VectorXd& _positions, State* _state) const;

  void convertStateToPositions(
    const State* _state, Eigen::VectorXd& _positions) const;

  /// Gets the positions of getMetaSkeleton() and store them in _state.
  void getState(State* _state) const;

  /// Wrapper for getStateFromMetaSkeleton that returns a ScopedState.
  ScopedState getScopedStateFromMetaSkeleton() const;

  /// Sets the MetaSkeleton's positions to the values stored in _state.
  void setState(const State* _state);

private:
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

using MetaSkeletonStateSpacePtr = std::shared_ptr<MetaSkeletonStateSpace>;

/// Create a JointStateSpace for a Joint whose type is known at compile time.
template <class JointType>
std::unique_ptr<JointStateSpace> createJointStateSpaceFor(JointType* _joint);

/// Create a JointStateSpace for an aribtrary Joint. This is significantly
/// slower than createJointStateSpaceFor(), so only use this function if the
/// Joint's type is not known at compile time.
std::unique_ptr<JointStateSpace> createJointStateSpace(
  ::dart::dynamics::Joint* _joint);

} // namespace dart
} // namespace statespace
} // namespace aikido

#include "detail/MetaSkeletonStateSpace.hpp"

#endif // ifndef AIKIDO_STATESPACE_METASKELETONSTATESPACE_HPP_
