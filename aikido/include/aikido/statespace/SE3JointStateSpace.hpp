#ifndef AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
#include "SE3StateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {

class SE3JointStateSpace : public SE3StateSpace, public JointStateSpace
{
public:
  using SE3StateSpace::State;

  SE3JointStateSpace(dart::dynamics::Joint* _joint);

  void getState(StateSpace::State* _state) const;

  void setState(const StateSpace::State* _state) const;
};


} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
