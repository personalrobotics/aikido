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
  explicit WorldStateSaver(WorldPtr world);

  virtual ~WorldStateSaver();

  // WorldStateSaver is uncopyable, must use std::move
  WorldStateSaver(const WorldStateSaver&) = delete;
  WorldStateSaver& operator=(const WorldStateSaver&)
      = delete;

  WorldStateSaver(WorldStateSaver&&) = default;
  WorldStateSaver& operator=(WorldStateSaver&&)
      = default;

private:
  WorldPtr mWorld;
  World::State mWorldState; 
};

} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_WORLDSTATESAVER_HPP_
