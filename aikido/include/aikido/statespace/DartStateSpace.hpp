#ifndef AIKIDO_STATESPACE_DARTSTATESPACE_H_
#define AIKIDO_STATESPACE_DARTSTATESPACE_H_
#include <dart/dynamics/dynamics.h>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>

namespace aikido {
namespace statespace {


class JointStateSpace : public virtual StateSpace
{
public:
  explicit JointStateSpace(dart::dynamics::Joint* _joint);

  virtual ~JointStateSpace() = default;

  dart::dynamics::Joint* getJoint();

  virtual void getState(StateSpace::State* _state) = 0;
  virtual void setState(const StateSpace::State* _state) = 0;

protected:
  dart::dynamics::Joint* mJoint;
};


/// StateSpace that represents the configuration space of a MetaSkeleton.
class MetaSkeletonStateSpace : public CompoundStateSpace
{
public:
  using CompoundStateSpace::State;
  using CompoundStateSpace::ScopedState;

  /// Create a new MetaSkeletonStateSpace.
  static std::shared_ptr<MetaSkeletonStateSpace> create(
    dart::dynamics::MetaSkeletonPtr _metaskeleton);
  
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

protected:
  MetaSkeletonStateSpace(dart::dynamics::MetaSkeletonPtr _metaskeleton,
    std::vector<StateSpacePtr> _stateSpaces,
    std::vector<std::shared_ptr<JointStateSpace>> _jointSpaces);

private:
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  std::vector<std::shared_ptr<JointStateSpace>> mJointSpaces;
};

} // namespace statespace
} // namespace aikido

#include "detail/DartStateSpace.hpp"

#endif // ifndef AIKIDO_STATESPACE_DARTSTATESPACE_H_
