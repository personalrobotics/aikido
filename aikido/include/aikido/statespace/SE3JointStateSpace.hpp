#ifndef AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
#include "RealVectorStateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {

class SE3JointStateSpace : public SE3StateSpace, public JointStateSpace
{
public:
  using SE3StateSpace::State;

  explicit SE3JointStateSpace(dart::dynamics::Joint* _joint)
    : JointStateSpace(_joint)
    , SE3StateSpace()
    // This is necessary because of virtual inheritance.
    , CompoundStateSpace({
        std::make_shared<SO3StateSpace>(),
        std::make_shared<RealVectorStateSpace>(3)
      })
  {
  }

  void getState(StateSpace::State* _state) const override
  {
    setIsometry(static_cast<State*>(_state),
      dart::dynamics::FreeJoint::convertToTransform(
        mJoint->getPositions()));
  }

  void setState(const StateSpace::State* _state) const override
  {
    mJoint->setPositions(
      dart::dynamics::FreeJoint::convertToPositions(
        getIsometry(static_cast<const SE3StateSpace::State*>(_state))));
  }
};


} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
