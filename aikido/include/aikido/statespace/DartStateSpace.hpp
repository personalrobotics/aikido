#ifndef AIKIDO_STATESPACE_DARTSTATESPACE_H_
#define AIKIDO_STATESPACE_DARTSTATESPACE_H_
#include <dart/dynamics/dynamics.h>
#include <aikido/statespace/CompoundStateSpace.hpp>

namespace aikido {
namespace statespace {


class JointStateSpace
{
public:
  JointStateSpace(
    std::shared_ptr<StateSpace> _space, dart::dynamics::Joint* _joint);

  virtual ~JointStateSpace() = default;

  const std::shared_ptr<StateSpace>& getStateSpace();
  dart::dynamics::Joint* getJoint();

  virtual void getState(StateSpace::State* _state) = 0;
  virtual void setState(const StateSpace::State* _state) = 0;

protected:
  std::shared_ptr<StateSpace> mStateSpace;
  dart::dynamics::Joint* mJoint;
};


class MetaSkeletonStateSpace : public CompoundStateSpace
{
public:
  using CompoundStateSpace::State;
  using CompoundStateSpace::ScopedState;

  static std::shared_ptr<MetaSkeletonStateSpace> create(
    dart::dynamics::MetaSkeletonPtr _metaskeleton);
  
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton() const;

  void getStateFromMetaSkeleton(State* _state) const;

  ScopedState getScopedStateFromMetaSkeleton() const;

  void setStateOnMetaSkeleton(const State* _state);

protected:
  MetaSkeletonStateSpace(dart::dynamics::MetaSkeletonPtr _metaskeleton,
    std::vector<StateSpacePtr> _stateSpaces,
    std::vector<std::unique_ptr<JointStateSpace>> _jointSpaces);

private:
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  std::vector<std::unique_ptr<JointStateSpace>> mJointSpaces;
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_DARTSTATESPACE_H_
