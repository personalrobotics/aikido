#ifndef AIKIDO_STATESPACE_SO2JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_SO2JOINTSTATESPACE_H_
#include "SO2StateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {

class SO2JointStateSpace : public SO2StateSpace, public JointStateSpace
{
public:
  using SO2StateSpace::State;

  explicit SO2JointStateSpace(dart::dynamics::Joint* _joint);

  void getState(StateSpace::State* _state) const override;

  void setState(const StateSpace::State* _state) const override;
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_SO2JOINTSTATESPACE_H_
