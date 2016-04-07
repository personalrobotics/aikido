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

  explicit RealVectorJointStateSpace(dart::dynamics::Joint* _joint);

  void getState(StateSpace::State* _state) const;

  void setState(const StateSpace::State* _state) const;
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_REALVECTORJOINTSTATESPACE_H_
