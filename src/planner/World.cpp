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
    const auto clonedSkeleton = mSkeletons[i]->clone();
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
bool World::hasSkeleton(const dart::dynamics::SkeletonPtr& skeleton) const
{
  return std::find(mSkeletons.begin(), mSkeletons.end(), skeleton)
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
  auto it = std::find(mSkeletons.begin(), mSkeletons.end(), skeleton);
  if (it == mSkeletons.end())
  {
    std::cout << "[World::removeSkeleton] Skeleton [" << skeleton->getName()
              << "] is not in the world." << std::endl;
    return;
  }

  // Remove skeleton from mSkeletons
  mSkeletons.erase(it);

  mSkeletonNameManager.removeName(skeleton->getName());
}

//==============================================================================
dart::dynamics::SimpleFramePtr World::getSimpleFrame(std::size_t index) const
{
  if (index < mSimpleFrames.size())
    return mSimpleFrames[index];

  return nullptr;
}

//==============================================================================
dart::dynamics::SimpleFramePtr World::getSimpleFrame(
    const std::string& name) const
{
  return mSimpleFrameNameManager.getObject(name);
}

//==============================================================================
bool World::hasSimpleFrame(const dart::dynamics::SimpleFramePtr& frame) const
{
  return std::find(mSimpleFrames.begin(), mSimpleFrames.end(), frame)
         != mSimpleFrames.end();
}

//==============================================================================
std::size_t World::getNumSimpleFrames() const
{
  return mSimpleFrames.size();
}

//==============================================================================
std::string World::addSimpleFrame(const dart::dynamics::SimpleFramePtr& frame)
{
  if (!frame)
  {
    std::cout << "Attempted to remove nullptr SimpleFrame from world";
    return "";
  }

  if (std::find(mSimpleFrames.begin(), mSimpleFrames.end(), frame)
      != mSimpleFrames.end())
  {
    std::cout << "[World::addFrame] SimpleFrame named [" << frame->getName()
           << "] is already in the world.\n";
    return frame->getName();
  }

  mSimpleFrames.push_back(frame);

  frame->setName(
      mSimpleFrameNameManager.issueNewNameAndAdd(frame->getName(), frame));

  return frame->getName();
}

//==============================================================================
void World::removeSimpleFrame(const dart::dynamics::SimpleFramePtr& frame)
{
  if (!frame)
  {
    std::cout << "Attempted to remove nullptr SimpleFrame from world";
    return;
  }

  auto it = std::find(mSimpleFrames.begin(), mSimpleFrames.end(), frame);
  if (it == mSimpleFrames.end())
  {
    std::cout << "[World::removeFrame] Frame named [" << frame->getName()
           << "] is not in the world.\n";
    return;
  }

  // Remove the frame
  mSimpleFrames.erase(it);

  // Remove from NameManager
  mSimpleFrameNameManager.removeName(frame->getName());
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
