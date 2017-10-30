#include <dart/dart.hpp>
#include <aikido/rviz/FrameMarker.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/rviz/SkeletonMarker.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

using aikido::rviz::InteractiveMarkerViewer;

namespace aikido {
namespace rviz {

//==============================================================================
WorldInteractiveMarkerViewer::WorldInteractiveMarkerViewer(
    aikido::planner::WorldPtr env,
    const std::string& topicNamespace,
    const std::string& frameId)
  : InteractiveMarkerViewer(topicNamespace, frameId), mWorld(std::move(env))
{
  // Do nothing
}

//==============================================================================
WorldInteractiveMarkerViewer::~WorldInteractiveMarkerViewer()
{
  // Do nothing
}

//==============================================================================
void WorldInteractiveMarkerViewer::setAutoUpdate(bool flag)
{
  mUpdating.store(flag, std::memory_order_release);

  const bool isRunning = mRunning.exchange(flag);
  if (flag && !isRunning)
    mThread = std::thread(&WorldInteractiveMarkerViewer::autoUpdate, this);
}

//==============================================================================
void WorldInteractiveMarkerViewer::autoUpdate()
{
  ros::Rate rate(30);

  while (mRunning.load() && ros::ok())
  {
    update();
    rate.sleep();
  }

  mRunning.store(false);
}

//==============================================================================
void WorldInteractiveMarkerViewer::update()
{
  std::lock_guard<std::mutex> lock(mMutex);

  // Update SkeletonMarkers
  for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i)
  {
    const dart::dynamics::SkeletonPtr skeleton = mWorld->getSkeleton(i);

    if (skeleton)
    {
      // Either a new SkeletonMarker or a previously-inserted SkeletonMarker
      auto result = mSkeletonMarkers.emplace(
          skeleton, CreateSkeletonMarker(skeleton, mFrameId));

      std::unique_lock<std::mutex> lock(skeleton->getMutex(), std::try_to_lock);
      if (lock.owns_lock())
        result.first->second->update();
    }
  }

  // Clear removed skeletons
  auto it = std::begin(mSkeletonMarkers);
  while (it != std::end(mSkeletonMarkers))
  {
    // Skeleton still exists in the World, do nothing.
    if (mWorld->getSkeleton(it->first))
    {
      ++it;
    }
    // Skeleton does not exist. Delete our existing ShapeFrameMarker.
    else
    {
      it = mSkeletonMarkers.erase(it);
    }
  }

  // TODO: Merge this into a unified update loop.
  for (const FrameMarkerPtr& marker : mFrameMarkers)
    marker->update();

  mMarkerServer.applyChanges();
}

} // namespace rviz
} // namespace aikido
