#ifndef AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
#include "SE3StateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {

/// Wrap a 6-DOF joint in a SE3StateSpace.
class SE3JointStateSpace : public SE3StateSpace, public JointStateSpace
{
public:
  using SE3StateSpace::State;

  explicit SE3JointStateSpace(dart::dynamics::FreeJoint* _joint);

  // Documentation inherited.
  void getState(StateSpace::State* _state) const;

  // Documentation inherited.
  void setState(const StateSpace::State* _state) const;
};


} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
