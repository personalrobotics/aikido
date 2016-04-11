#ifndef AIKIDO_STATESPACE_METASKELETONSTATESPACE_H_
#define AIKIDO_STATESPACE_METASKELETONSTATESPACE_H_
#include <dart/dynamics/dynamics.h>
#include "CompoundStateSpace.hpp"
#include "JointStateSpace.hpp"
#include "../constraint/Sampleable.hpp"

namespace aikido {
namespace statespace {

/// StateSpace that represents the configuration space of a MetaSkeleton.
class MetaSkeletonStateSpace : public CompoundStateSpace
{
public:
  using CompoundStateSpace::State;
  using CompoundStateSpace::ScopedState;
  using SampleableConstraintPtr = constraint::SampleableConstraintPtr;

  /// Create a StateSpace for the Joints in the MetaSkeleton.
  MetaSkeletonStateSpace(dart::dynamics::MetaSkeletonPtr _metaskeleton);

  /// Get the MetaSkeleton associated with this StateSpace.
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton() const;

  /// Get the subspace associated with a joint.
  template <class Space = StateSpace>
  const StateSpace* getJointSpace(const dart::dynamics::Joint* _joint) const;

  /// Gets the positions of getMetaSkeleton() and store them in _state.
  void getStateFromMetaSkeleton(State* _state) const;

  /// Wrapper for getStateFromMetaSkeleton that returns a ScopedState.
  ScopedState getScopedStateFromMetaSkeleton() const;

  /// Sets the MetaSkeleton's positions to the values stored in _state.
  void setStateOnMetaSkeleton(const State* _state);

  /// Sample uniformly at random from this state space.
  SampleableConstraintPtr createSampleableConstraint(
    std::unique_ptr<util::RNG> _rng) const;

private:
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

using MetaSkeletonStateSpacePtr = std::shared_ptr<MetaSkeletonStateSpace>;

/// Create a JointStateSpace for a Joint whose type is known at compile time.
template <class JointType>
std::unique_ptr<JointStateSpace> createJointStateSpaceFor(JointType* _joint);

/// Create a JointStateSpace for an aribtrary Joint. This is significantly
/// slower than createJointStateSpaceFor(), so only use this function if the
/// Joint's type is not known at compile time.
std::unique_ptr<JointStateSpace> createJointStateSpace(
  dart::dynamics::Joint* _joint);

} // namespace statespace
} // namespace aikido

#include "detail/MetaSkeletonStateSpace.hpp"

#endif // ifndef AIKIDO_STATESPACE_METASKELETONSTATESPACE_H_
