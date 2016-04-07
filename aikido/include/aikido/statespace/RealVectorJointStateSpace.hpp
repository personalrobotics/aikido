#ifndef AIKIDO_STATESPACE_REALVECTORJOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_REALVECTORJOINTSTATESPACE_H_
#include "RealVectorStateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {

class RealVectorJointStateSpace
  : public RealVectorStateSpace
  , public JointStateSpace
{
public:
  using RealVectorStateSpace::State;

  explicit RealVectorJointStateSpace(dart::dynamics::Joint* _joint)
    : RealVectorStateSpace(_joint->getNumDofs())
    , JointStateSpace(_joint)
  {
  }

  void getState(StateSpace::State* _state) const override
  {
    setValue(static_cast<State*>(_state), mJoint->getPositions());
  }

  void setState(const StateSpace::State* _state) const override
  {
    mJoint->setPositions(getValue(static_cast<const State*>(_state)));
  }
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_REALVECTORJOINTSTATESPACE_H_
