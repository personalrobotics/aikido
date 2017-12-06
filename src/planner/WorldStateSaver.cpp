#include <aikido/planner/WorldStateSaver.hpp>

namespace aikido {
namespace planner {

WorldStateSaver::WorldStateSaver(World* const world) : mWorld{world}
{
  if (!world)
    throw std::invalid_argument("World must not be nullptr.");

  mWorldState = mWorld->getState();
}

WorldStateSaver::~WorldStateSaver()
{
  mWorld->setState(mWorldState);
}

} // namespace planner
} // namespace aikido
