#include <aikido/planner/World.hpp>

namespace aikido {
namespace planner {

dart::common::NameManager<World*> World::worldNameManager{"World", "world"};

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
  World::worldNameManager.removeName(mName);
}

//==============================================================================
WorldPtr World::create(const std::string& name)
{
  WorldPtr world(new World(name));
  return world;
}

//==============================================================================
WorldPtr World::clone(const std::string& newName) const
{
  WorldPtr worldClone(new World(newName.empty() ? mName : newName));

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
    mName = World::worldNameManager.issueNewNameAndAdd(newName, this);
  else
    mName = World::worldNameManager.changeObjectName(this, newName);
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

} // namespace planner
} // namespace aikido
