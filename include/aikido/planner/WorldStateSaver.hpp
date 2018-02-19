#ifndef AIKIDO_PLANNER_WORLDSTATESAVER_HPP_
#define AIKIDO_PLANNER_WORLDSTATESAVER_HPP_

#include "aikido/planner/World.hpp"

namespace aikido {
namespace planner {

/// RAII class to save and restore a World's state.
class WorldStateSaver
{
public:
  /// Options to specify what WorldStateSaver should save.
  enum Options
  {
    CONFIGURATIONS = 1 << 0,
  };

  /// Construct a WorldStateSaver and save the current state of the \c World.
  /// This state will be restored when WorldStateSaver is destructed.
  ///
  /// \param[in] world World to save state from and restore to.
  /// \param[in] options Options to specify what should be saved
  explicit WorldStateSaver(const World* world, int options = CONFIGURATIONS);

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
