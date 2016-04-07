#ifndef AIKIDO_STATESPACE_JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_JOINTSTATESPACE_H_
#include <dart/dynamics/dynamics.h>
#include "StateSpace.hpp"

namespace aikido {
namespace statespace {

class JointStateSpace : public virtual StateSpace
{
public:
  explicit JointStateSpace(dart::dynamics::Joint* _joint);

  virtual ~JointStateSpace() = default;

  dart::dynamics::Joint* getJoint() const;

  virtual void getState(StateSpace::State* _state) const = 0;
  virtual void setState(const StateSpace::State* _state) const = 0;

protected:
  dart::dynamics::Joint* mJoint;
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_JOINTSTATESPACE_H_
