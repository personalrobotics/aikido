#ifndef AIKIDO_PLANNER_WORLD_HPP_
#define AIKIDO_PLANNER_WORLD_HPP_

#include <string>
#include <dart/dart.hpp>

namespace aikido {
namespace planner {

class World
{
public:
  /// Construct a kinematic World.
  World(const std::string& name);

  virtual ~World() = default;

  /// Create a clone of this World. All Skeletons will be copied over.
  /// \param newName Name for the cloned World
  std::shared_ptr<World> clone(const std::string& newName) const;

  /// Create a clone of this World with the same name.
  std::shared_ptr<World> clone() const;

  /// Set the name of this World
  const std::string& setName(const std::string& newName);

  /// Get the name of this World
  const std::string& getName() const;

  /// Find a Skeleton by index
  dart::dynamics::SkeletonPtr getSkeleton(std::size_t i) const;

  /// Find a Skeleton by name
  dart::dynamics::SkeletonPtr getSkeleton(const std::string& name) const;

  /// Get the number of Skeletons
  std::size_t getNumSkeletons() const;

  /// Add a Skeleton to this World
  std::string addSkeleton(const dart::dynamics::SkeletonPtr& skeleton);

  /// Remove a Skeleton from this World
  void removeSkeleton(const dart::dynamics::SkeletonPtr& skeleton);

  // TODO: Add methods for registering callbacks?

  // Get the mutex that protects the state of this World.
  std::mutex& getMutex() const;

protected:
  /// Name of this World
  std::string mName;

  /// Skeletons in this World
  std::vector<dart::dynamics::SkeletonPtr> mSkeletons;

  mutable std::mutex mMutex;
};

typedef std::shared_ptr<World> WorldPtr;

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_WORLD_HPP_
