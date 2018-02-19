#include "aikido/planner/WorldStateSaver.hpp"

namespace aikido {
namespace planner {

WorldStateSaver::WorldStateSaver(World* world, int options)
  : mWorld{std::move(world)}, mOptions{options}
{
  if (!mWorld)
    throw std::invalid_argument("World must not be nullptr.");

  if (mOptions & Options::CONFIGURATIONS)
    mWorldState = mWorld->getState();
}

WorldStateSaver::~WorldStateSaver()
{
  if (mOptions & Options::CONFIGURATIONS)
    mWorld->setState(mWorldState);
}

} // namespace planner
} // namespace aikido
