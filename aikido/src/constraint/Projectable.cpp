#include <aikido/constraint/Projectable.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
bool Projectable::project(statespace::StateSpace::State* _s) const
{
  auto space = getStateSpace();
  auto state = space->createState();
  space->copyState(state, _s);
  return project(state, _s);
}

} // namespace constraint
} // namespace aikido
