#include <sstream>
#include <aikido/statespace/SO2JointStateSpace.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
SO2JointStateSpace::SO2JointStateSpace(dart::dynamics::Joint* _joint)
  : JointStateSpace(_joint)
  , SO2StateSpace()
{
  if (_joint->getNumDofs() != 1)
  {
    std::stringstream msg;
    msg << "Joint '" << _joint->getName() << "' has " << _joint->getNumDofs()
        << ", but SO2JointStateSpace only supports one DOF.";
    throw std::runtime_error(msg.str());
  }
}

//=============================================================================
void SO2JointStateSpace::getState(StateSpace::State* _state) const
{
  setAngle(static_cast<State*>(_state), mJoint->getPosition(0));
}

//=============================================================================
void SO2JointStateSpace::setState(const StateSpace::State* _state) const
{
  mJoint->setPosition(0, getAngle(static_cast<const State*>(_state)));
}

} // namespace statespace
} // namespace aikido
