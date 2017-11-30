#include <aikido/planner/WorldStateSaver.hpp>

namespace aikido {
namespace planner {

WorldStateSaver::WorldStateSaver(
    WorldPtr world)
  : mWorld{world}
{
  mWorldState = mWorld->getState();
}

WorldStateSaver::~WorldStateSaver()
{
  mWorld->setState(mWorldState);
}

} // namespace planner
} // namespace aikido
