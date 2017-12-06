#ifndef AIKIDO_PLANNER_WORLDSTATESAVER_HPP_
#define AIKIDO_PLANNER_WORLDSTATESAVER_HPP_
#include "World.hpp"

namespace aikido {
namespace planner {

/// RAII class to save and restore a World's state.
class WorldStateSaver
{
public:
  /// Construct a WorldStateSaver and save the current state of the
  /// \c World. This state will be restored when
  /// WorldStateSaver is destructed.
  ///
  /// \param _space WorldState to save/restore
  explicit WorldStateSaver(World* const world);

  virtual ~WorldStateSaver();

private:
  World* mWorld;
  World::State mWorldState;
};

} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_WORLDSTATESAVER_HPP_
