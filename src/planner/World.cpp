#include <dart/common/StlHelpers.hpp>
#include <aikido/planner/World.hpp>

namespace aikido {
namespace planner {

dart::common::NameManager<World*> World::mWorldNameManager{"World", "world"};

//==============================================================================
World::World(const std::string& name)
{
  setName(name);

  mSkeletonNameManager.setManagerName("World::Skeleton | " + mName);
  mSkeletonNameManager.setDefaultName("skeleton");
}

//==============================================================================
World::~World()
{
  World::mWorldNameManager.removeName(mName);
}

//==============================================================================
std::unique_ptr<World> World::create(const std::string& name)
{
  std::unique_ptr<World> world(new World(name));
  return world;
}

//==============================================================================
std::unique_ptr<World> World::clone(const std::string& newName) const
{
  std::unique_ptr<World> worldClone(
      new World(newName.empty() ? mName : newName));

  // Clone and add each Skeleton
  worldClone->mSkeletons.reserve(mSkeletons.size());
  for (std::size_t i = 0; i < mSkeletons.size(); ++i)
  {
#if DART_VERSION_AT_LEAST(6,7,0)
    const auto clonedSkeleton = mSkeletons[i]->cloneSkeleton();
#else
    const auto clonedSkeleton = mSkeletons[i]->clone();
#endif
    clonedSkeleton->setConfiguration(mSkeletons[i]->getConfiguration());
    worldClone->addSkeleton(std::move(clonedSkeleton));
  }

  return worldClone;
}

//==============================================================================
std::string World::setName(const std::string& newName)
{
  if (mName.empty())
    mName = World::mWorldNameManager.issueNewNameAndAdd(newName, this);
  else
    mName = World::mWorldNameManager.changeObjectName(this, newName);
  return mName;
}

//==============================================================================
const std::string& World::getName() const
{
  return mName;
}

//==============================================================================
dart::dynamics::SkeletonPtr World::getSkeleton(std::size_t i) const
{
  if (i < mSkeletons.size())
    return mSkeletons[i];

  return nullptr;
}

//==============================================================================
dart::dynamics::SkeletonPtr World::getSkeleton(const std::string& name) const
{
  return mSkeletonNameManager.getObject(name);
}

//==============================================================================
bool World::hasSkeleton(const dart::dynamics::SkeletonPtr& skel) const
{
  return std::find(mSkeletons.begin(), mSkeletons.end(), skel)
         != mSkeletons.end();
}

//==============================================================================
std::size_t World::getNumSkeletons() const
{
  return mSkeletons.size();
}

//==============================================================================
std::string World::addSkeleton(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton)
  {
    std::cout << "[World::addSkeleton] Attempting to add a nullptr Skeleton to "
              << "the world!" << std::endl;
    return "";
  }

  std::lock_guard<std::mutex> lock(mMutex);

  // If mSkeletons already has skeleton, then do nothing.
  if (std::find(mSkeletons.begin(), mSkeletons.end(), skeleton)
      != mSkeletons.end())
  {
    std::cout << "[World::addSkeleton] Skeleton named [" << skeleton->getName()
              << "] is already in the world." << std::endl;
    return skeleton->getName();
  }

  mSkeletons.push_back(skeleton);

  skeleton->setName(
      mSkeletonNameManager.issueNewNameAndAdd(skeleton->getName(), skeleton));

  return skeleton->getName();
}

//==============================================================================
void World::removeSkeleton(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton)
  {
    std::cout << "[World::removeSkeleton] Attempting to remove a nullptr "
              << "Skeleton from the world!" << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(mMutex);

  // If mSkeletons doesn't have skeleton, then do nothing.
  auto skelIt = std::find(mSkeletons.begin(), mSkeletons.end(), skeleton);
  if (skelIt == mSkeletons.end())
  {
    std::cout << "[World::removeSkeleton] Skeleton [" << skeleton->getName()
              << "] is not in the world." << std::endl;
    return;
  }

  // Remove skeleton from mSkeletons
  mSkeletons.erase(skelIt);

  mSkeletonNameManager.removeName(skeleton->getName());
}

//==============================================================================
std::mutex& World::getMutex() const
{
  return mMutex;
}

//==============================================================================
bool World::State::operator==(const State& other) const
{
  return configurations == other.configurations;
}

//==============================================================================
bool World::State::operator!=(const State& other) const
{
  return !(*this == other);
}

//==============================================================================
World::State World::getState() const
{
  World::State state;

  for (const auto& skeleton : mSkeletons)
    state.configurations[skeleton->getName()] = skeleton->getConfiguration();

  return state;
}

//==============================================================================
void World::setState(const World::State& state)
{
  if (state.configurations.size() != mSkeletons.size())
    throw std::invalid_argument(
        "World::State and this World do not have the same number of "
        "skeletons.");

  for (const auto& skeleton : mSkeletons)
  {
    auto name = skeleton->getName();
    auto it = state.configurations.find(name);
    if (it == state.configurations.end())
      throw std::invalid_argument(
          "Skeleton " + name + " does not exist in state.");
  }

  for (const auto& skeleton : mSkeletons)
  {
    std::lock_guard<std::mutex> lock(skeleton->getMutex());
    skeleton->setConfiguration(state.configurations.at(skeleton->getName()));
  }
}

} // namespace planner
} // namespace aikido
