#ifndef AIKIDO_STATESPACE_SO2JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_SO2JOINTSTATESPACE_H_
#include "RealVectorStateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {

class SO2JointStateSpace : public SO2StateSpace, public JointStateSpace
{
public:
  using SO2StateSpace::State;

  explicit SO2JointStateSpace(dart::dynamics::Joint* _joint)
    : JointStateSpace(_joint)
    , SO2StateSpace()
  {
    //assert(_joint->getNumDofs() == 1);
  }

  void getState(StateSpace::State* _state) const override
  {
    setAngle(static_cast<State*>(_state), mJoint->getPosition(0));
  }

  void setState(const StateSpace::State* _state) const override
  {
    mJoint->setPosition(0, getAngle(static_cast<const State*>(_state)));
  }
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_SO2JOINTSTATESPACE_H_
