#include "aikido/planner/WorldStateSaver.hpp"

namespace aikido {
namespace planner {

WorldStateSaver::WorldStateSaver(World* const world, int options)
  : mWorld{world}, mOptions{options}
{
  if (!world)
    throw std::invalid_argument("World must not be nullptr.");

  if (mOptions & WorldStateSaverOptions::CONFIGURATIONS)
    mWorldState = mWorld->getState();
}

WorldStateSaver::~WorldStateSaver()
{
  if (mOptions & WorldStateSaverOptions::CONFIGURATIONS)
    mWorld->setState(mWorldState);
}

} // namespace planner
} // namespace aikido
