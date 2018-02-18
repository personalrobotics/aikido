#ifndef AIKIDO_PLANNER_WORLDSTATESAVER_HPP_
#define AIKIDO_PLANNER_WORLDSTATESAVER_HPP_

#include "aikido/planner/World.hpp"

namespace aikido {
namespace planner {

/// Options to specify what WorldStateSaver should save.
enum WorldStateSaverOptions
{
  CONFIGURATIONS = 1 << 0,
};

/// RAII class to save and restore a World's state.
class WorldStateSaver
{
public:
  /// Construct a WorldStateSaver and save the current state of the \c World.
  /// This state will be restored when WorldStateSaver is destructed.
  ///
  /// \param world World to save state from and restore to.
  /// \param options Options to specify what should be saved
  explicit WorldStateSaver(World* const world, int options = CONFIGURATIONS);

  virtual ~WorldStateSaver();

private:
  /// World to save the state of
  World* mWorld;

  /// Options to specify what should be saved
  int mOptions;

  /// Saved state
  World::State mWorldState;
};

} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_WORLDSTATESAVER_HPP_
