#include <aikido/rviz/InteractiveMarkerViewer.hpp>

#include <dart/common/StlHelpers.hpp>
#include <dart/dart.hpp>
#include <aikido/rviz/FrameMarker.hpp>
#include <aikido/rviz/SkeletonMarker.hpp>
#include <aikido/rviz/TrajectoryMarker.hpp>

using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using interactive_markers::InteractiveMarkerServer;

namespace aikido {
namespace rviz {

//==============================================================================
InteractiveMarkerViewer::InteractiveMarkerViewer(
    const std::string& topicNamespace, const std::string& frameId)
  : mMarkerServer(topicNamespace, "", true)
  , mTrajectoryNameManager("Trajectory Name Manager", "Trajectory")
  , mRunning(false)
  , mUpdating(false)
  , mFrameId(frameId)
{
  mTrajectoryNameManager.setPattern("Frame[%s(%d)]");
}

//==============================================================================
InteractiveMarkerViewer::~InteractiveMarkerViewer()
{
  mRunning.store(false, std::memory_order_release);
  mThread.join();
}

//==============================================================================
InteractiveMarkerServer& InteractiveMarkerViewer::marker_server()
{
  return mMarkerServer;
}

//==============================================================================
SkeletonMarkerPtr InteractiveMarkerViewer::addSkeleton(
    const SkeletonPtr& skeleton)
{
  std::lock_guard<std::mutex> lock(mMutex);
  const SkeletonMarkerPtr marker = CreateSkeletonMarker(skeleton, mFrameId);
  mSkeletonMarkers.insert(marker);
  return marker;
}

//==============================================================================
FrameMarkerPtr InteractiveMarkerViewer::addFrame(
    dart::dynamics::Frame* frame, double length, double thickness, double alpha)
{
  std::lock_guard<std::mutex> lock(mMutex);
  const FrameMarkerPtr marker = std::make_shared<FrameMarker>(
      &mMarkerServer, frame, mFrameId, length, thickness, alpha);
  mFrameMarkers.insert(marker);
  return marker;
}

//==============================================================================
TSRMarkerPtr InteractiveMarkerViewer::addTSRMarker(
    const constraint::dart::TSR& tsr, int nSamples, const std::string& basename)
{
  using dart::dynamics::Frame;
  using dart::dynamics::SimpleFrame;

  auto sampler = tsr.createSampleGenerator();
  auto state = tsr.getSE3()->createState();

  std::string name;
  if (basename.empty())
  {
    std::ostringstream ost;
    ost << &tsr;
    name = ost.str();
  }
  else
  {
    name = basename;
  }

  std::vector<std::unique_ptr<SimpleFrame>> tsrFrames;
  tsrFrames.reserve(nSamples);

  for (int i = 0; i < nSamples && sampler->canSample(); ++i)
  {
    auto sampled = sampler->sample(state);
    assert(sampled);
    DART_UNUSED(sampled);

    std::stringstream ss;
    ss << "TSRMarker[" << name << "].frame[" << i << "]";

    auto tsrFrame = dart::common::make_unique<SimpleFrame>(
        Frame::World(), ss.str(), state.getIsometry());
    addFrame(tsrFrame.get());
    tsrFrames.emplace_back(std::move(tsrFrame));
  }

  auto tsrMarker = std::make_shared<TSRMarker>(std::move(tsrFrames));
  return tsrMarker;
}

//==============================================================================
TrajectoryMarkerPtr InteractiveMarkerViewer::addTrajectoryMarker(
    trajectory::ConstTrajectoryPtr trajectory,
    dart::dynamics::MetaSkeletonPtr skeleton,
    const dart::dynamics::Frame& frame,
    const Eigen::Vector4d& rgba,
    double thickness,
    std::size_t numLineSegments)
{
  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock);
  auto marker = std::make_shared<TrajectoryMarker>(
      &mMarkerServer,
      mFrameId,
      mTrajectoryNameManager.issueNewNameAndAdd("", trajectory),
      std::move(trajectory),
      std::move(skeleton),
      frame,
      rgba,
      thickness,
      numLineSegments);
  mTrajectoryMarkers.insert(marker);

  return marker;
}

bool InteractiveMarkerViewer::removeTrajectoryMarker(const TrajectoryMarkerPtr& trajectoryMarkerToRemove) {
  return mTrajectoryMarkers.erase(trajectoryMarkerToRemove) > 0;
}

//==============================================================================
SkeletonMarkerPtr InteractiveMarkerViewer::CreateSkeletonMarker(
    const SkeletonPtr& skeleton, const std::string& frameId)
{
  return std::make_shared<SkeletonMarker>(
      nullptr, &mMarkerServer, skeleton, frameId);
}

//==============================================================================
void InteractiveMarkerViewer::setAutoUpdate(bool flag)
{
  mUpdating.store(flag, std::memory_order_release);

  const bool wasRunning = mRunning.exchange(flag);
  if (flag && !wasRunning)
    mThread = std::thread(&InteractiveMarkerViewer::autoUpdate, this);
  else if (!flag && wasRunning)
    mThread.join();
}

//==============================================================================
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

//==============================================================================
void InteractiveMarkerViewer::update()
{
  std::lock_guard<std::mutex> lock(mMutex);

  for (auto it = std::begin(mSkeletonMarkers);
       it != std::end(mSkeletonMarkers);)
  {
    const dart::dynamics::SkeletonPtr skeleton = (*it)->getSkeleton();

    if (skeleton)
    {
      std::unique_lock<std::mutex> skeleton_lock(
          skeleton->getMutex(), std::try_to_lock);
      if (skeleton_lock.owns_lock())
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
  for (const FrameMarkerPtr& marker : mFrameMarkers)
    marker->update();

  for (const auto& marker : mTrajectoryMarkers)
    marker->update();

  mMarkerServer.applyChanges();
}

} // namespace rviz
} // namespace aikido
