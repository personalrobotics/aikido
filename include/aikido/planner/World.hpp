#ifndef AIKIDO_PLANNER_WORLD_HPP_
#define AIKIDO_PLANNER_WORLD_HPP_

#include <string>
#include <dart/dart.hpp>

namespace aikido {
namespace planner {

/// A Kinematic world that contains a set of skeletons.
/// It is expected that worlds do not share the same skeletons.
class World
{
public:
  // Encapsulates the state of the World.
  struct State
  {
    std::vector<dart::dynamics::Skeleton::Configuration> configurations;

    /// Returns true if two world states are the same.
    /// \param other State to compare against.
    /// \return bool True if the two states are the same.
    bool equals(const State& other);
  };

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

  /// Returns true if the Skeleton is in this World.
  /// \param skel Desired Skeleton
  /// \return True if succeeded to find the Skeleton
  bool hasSkeleton(const dart::dynamics::SkeletonPtr& skel) const;

  /// Get the number of Skeletons
  std::size_t getNumSkeletons() const;

  /// Add a Skeleton to this World
  /// \param skeleton Skeleton to add to the World
  std::string addSkeleton(const dart::dynamics::SkeletonPtr& skeleton);

  /// Remove a Skeleton from this World
  /// \param skeleton Skeleton to remove from the World
  void removeSkeleton(const dart::dynamics::SkeletonPtr& skeleton);

  // TODO: Add methods for registering callbacks?

  /// Get the mutex that protects the state of this World.
  std::mutex& getMutex() const;

  /// Returns the state of this World.
  /// \return State
  World::State getState() const;

  /// Sets the state of this World to match State.
  /// The caller of this method MUST LOCK the mutex of this World.
  /// \param State State to set this world to.
  void setState(const World::State& State);

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
};

using WorldPtr = std::shared_ptr<World>;

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_WORLD_HPP_
