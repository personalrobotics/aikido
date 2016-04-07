#ifndef AIKIDO_STATESPACE_DARTSTATESPACE_H_
#define AIKIDO_STATESPACE_DARTSTATESPACE_H_
#include <dart/dynamics/dynamics.h>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>

namespace aikido {
namespace statespace {

/// StateSpace that represents the configuration space of a MetaSkeleton.
class MetaSkeletonStateSpace : public CompoundStateSpace
{
public:
  using CompoundStateSpace::State;
  using CompoundStateSpace::ScopedState;

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

private:
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

} // namespace statespace
} // namespace aikido

#include "detail/DartStateSpace.hpp"

#endif // ifndef AIKIDO_STATESPACE_DARTSTATESPACE_H_
