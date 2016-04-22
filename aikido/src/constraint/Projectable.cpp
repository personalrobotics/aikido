#include <aikido/constraint/Projectable.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
bool Projectable::project(const statespace::StateSpacePtr _space,
  statespace::StateSpace::State* _s) const
{
  auto state = _space->createState();
  _space->copyState(state, _s);
  return project(state, _s);
}

} // namespace constraint
} // namespace aikido
