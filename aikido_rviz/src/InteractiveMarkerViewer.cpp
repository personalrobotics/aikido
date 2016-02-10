#include <dart_rviz/FrameMarker.h>
#include <dart_rviz/SkeletonMarker.h>
#include <dart_rviz/InteractiveMarkerViewer.h>

using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using aikido::rviz::FrameMarker;
using aikido::rviz::FrameMarkerPtr;
using aikido::rviz::SkeletonMarker;
using aikido::rviz::SkeletonMarkerPtr;
using aikido::rviz::InteractiveMarkerViewer;
using interactive_markers::InteractiveMarkerServer;

InteractiveMarkerViewer::InteractiveMarkerViewer(
    std::string const &topicNamespace)
  : mMarkerServer(topicNamespace, "", true),
    mRunning(false),
    mUpdating(false)
{
}

InteractiveMarkerViewer::~InteractiveMarkerViewer()
{
  mRunning.store(false, std::memory_order_release);
  mThread.join();
}

InteractiveMarkerServer &InteractiveMarkerViewer::marker_server()
{
  return mMarkerServer;
}

SkeletonMarkerPtr InteractiveMarkerViewer::addSkeleton(
  SkeletonPtr const &skeleton)
{
  std::lock_guard<std::mutex> lock(mMutex);
  SkeletonMarkerPtr const marker = CreateSkeletonMarker(skeleton);
  mSkeletonMarkers.insert(marker);
  return marker;
}

FrameMarkerPtr InteractiveMarkerViewer::addFrame(
  dart::dynamics::Frame *frame, double length, double thickness, double alpha)
{
  std::lock_guard<std::mutex> lock(mMutex);
  FrameMarkerPtr const marker = std::make_shared<FrameMarker>(
    &mMarkerServer, frame, length, thickness, alpha);
  mFrameMarkers.insert(marker);
  return marker;
}

SkeletonMarkerPtr InteractiveMarkerViewer::CreateSkeletonMarker(
  SkeletonPtr const &skeleton)
{
  return std::make_shared<SkeletonMarker>(nullptr, &mMarkerServer, skeleton);
}

void InteractiveMarkerViewer::setAutoUpdate(bool _flag)
{
  mUpdating.store(_flag, std::memory_order_release);

  const bool isRunning = mRunning.exchange(_flag);
  if (_flag && !isRunning)
    mThread = std::thread(&InteractiveMarkerViewer::autoUpdate, this);
}

void InteractiveMarkerViewer::autoUpdate()
{
  ros::Rate rate(30);

  while (mRunning.load() && ros::ok())
  {
    update();
    rate.sleep();
  }

  mRunning.store(false);
}

void InteractiveMarkerViewer::update()
{
  std::lock_guard<std::mutex> lock(mMutex);

  for (auto it = std::begin(mSkeletonMarkers); it != std::end(mSkeletonMarkers); )
  {
    const dart::dynamics::SkeletonPtr skeleton = (*it)->getSkeleton();

    if (skeleton)
    {
      std::unique_lock<std::mutex> lock(skeleton->getMutex(), std::try_to_lock);

      if (lock.owns_lock())
        (*it)->update();

      ++it;
    }
    // Skeleton no longer exists. Delete the marker.
    else
    {
      it = mSkeletonMarkers.erase(it);
    }
  }

  // TODO: Merge this into a unified update loop.
  for (FrameMarkerPtr const &marker : mFrameMarkers) {
    marker->update();
  }

  mMarkerServer.applyChanges();
}
