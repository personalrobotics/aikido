#include <aikido/statespace/SO3StateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SE3JointStateSpace.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
SE3JointStateSpace::SE3JointStateSpace(dart::dynamics::FreeJoint* _joint)
  : JointStateSpace(_joint)
  , SE3StateSpace()
{
}

//=============================================================================
void SE3JointStateSpace::getState(StateSpace::State* _state) const
{
  setIsometry(static_cast<State*>(_state),
    dart::dynamics::FreeJoint::convertToTransform(
      mJoint->getPositions()));
}

//=============================================================================
void SE3JointStateSpace::setState(const StateSpace::State* _state) const
{
  mJoint->setPositions(
    dart::dynamics::FreeJoint::convertToPositions(
      getIsometry(static_cast<const SE3StateSpace::State*>(_state))));
}

} // namespace statespace
} // namespace aikido
