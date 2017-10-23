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
  /// \param name Name for the new World
  World(const std::string& name = "");

  virtual ~World();

  /// Create a new World inside of a shared_ptr
  /// \param name Name for the new World
  static std::unique_ptr<World> create(const std::string& name = "");

  /// Create a clone of this World. All Skeletons will be copied over.
  /// \param newName Name for the cloned World
  std::unique_ptr<World> clone(const std::string& newName = "") const;

  /// Set the name of this World
  /// \param newName New name for this World
  std::string setName(const std::string& newName);

  /// Get the name of this World
  const std::string& getName() const;

  /// Find a Skeleton by index
  /// \param i Index of desired Skeleton
  dart::dynamics::SkeletonPtr getSkeleton(std::size_t i) const;

  /// Find a Skeleton by name
  /// \param name Name of desired Skeleton
  dart::dynamics::SkeletonPtr getSkeleton(const std::string& name) const;

  /// Get the number of Skeletons
  std::size_t getNumSkeletons() const;

  /// Add a Skeleton to this World
  /// \param skeleton Skeleton to add to the World
  std::string addSkeleton(const dart::dynamics::SkeletonPtr& skeleton);

  /// Remove a Skeleton from this World
  /// \param skeleton Skeleton to remove from the World
  void removeSkeleton(const dart::dynamics::SkeletonPtr& skeleton);

  // TODO: Add methods for registering callbacks?

  /// Get the CollisionDetector for this World
  dart::collision::CollisionDetectorPtr getCollisionDetector() const;

  /// Create a CollisionGroup for this World
  dart::collision::CollisionGroupPtr createCollisionGroup();

  /// Create an empty NonColliding constraint for this World
  /// \param space Space for this NonColliding constraint
  /// \param collisionFilter Filter for this NonColliding constraint
  aikido::constraint::NonCollidingPtr createCollisionConstraint(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr space,
    std::shared_ptr<dart::collision::CollisionFilter> collisionFilter = nullptr);

  /// Get the mutex that protects the state of this World.
  std::mutex& getMutex() const;

protected:
  /// Name of this World
  std::string mName;

  /// Skeletons in this World
  std::vector<dart::dynamics::SkeletonPtr> mSkeletons;

  /// Mutex to protect this World
  mutable std::mutex mMutex;

  /// NameManager for keeping track of Worlds
  static dart::common::NameManager<World*> mWorldNameManager;

  /// NameManager for keeping track of Skeletons
  dart::common::NameManager<dart::dynamics::SkeletonPtr> mSkeletonNameManager;

  /// CollisionDetector for this World
  dart::collision::CollisionDetectorPtr mCollisionDetector;
};

using WorldPtr = std::shared_ptr<World>;

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_WORLD_HPP_
