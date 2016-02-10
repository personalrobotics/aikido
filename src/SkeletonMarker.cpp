#include <dart_rviz/SkeletonMarker.h>

using dart::dynamics::BodyNode;
using dart::dynamics::WeakBodyNodePtr;
using dart::dynamics::WeakSkeletonPtr;
using dart::dynamics::SkeletonPtr;
using dart::rviz::BodyNodeMarker;
using dart::rviz::BodyNodeMarkerPtr;
using dart::rviz::SkeletonMarker;
using interactive_markers::InteractiveMarkerServer;

SkeletonMarker::SkeletonMarker(ResourceServer *resourceServer,
                               InteractiveMarkerServer *markerServer,
                               WeakSkeletonPtr const &skeleton)
  : mSkeleton(skeleton)
  , mResourceServer(resourceServer)
  , mMarkerServer(markerServer)
  , mHasColor(false)
{
}

dart::dynamics::SkeletonPtr SkeletonMarker::getSkeleton() const
{
  return mSkeleton.lock();
}

bool SkeletonMarker::update()
{
  SkeletonPtr const skeleton = mSkeleton.lock();
  if (!skeleton) {
    return false;
  }

  for (BodyNode *const bodyNode : skeleton->getBodyNodes()) {
    // Lazily create a BodyNodeMarker.
    auto result = mBodyNodeMarkers.insert(
      std::make_pair(bodyNode, BodyNodeMarkerPtr()));
    BodyNodeMarkerPtr &bodyNodeMarker = result.first->second;

    if (result.second) {
      bodyNodeMarker = std::make_shared<BodyNodeMarker>(
        mResourceServer, mMarkerServer, WeakBodyNodePtr(bodyNode));

      if (mHasColor)
        bodyNodeMarker->SetColor(mColor);
    }

    // Update the BodyNodeMarker. If update() returns false, then the BodyNode
    // was deleted.
    if (!bodyNodeMarker->update()) {
      mBodyNodeMarkers.erase(result.first);
    }
  }
  return true;
}

BodyNodeMarkerPtr SkeletonMarker::GetBodyNodeMarker(
  dart::dynamics::BodyNode const *bodynode)
{
  const auto it = mBodyNodeMarkers.find(bodynode);
  if (it != std::end(mBodyNodeMarkers))
    return it->second;
  else
    throw std::runtime_error("There is no marker for this BodyNode.");
}

void SkeletonMarker::SetColor(Eigen::Vector4d const &color)
{
  mColor = color;
  mHasColor = true;

  for (auto const &it : mBodyNodeMarkers) {
    it.second->SetColor(color);
  }
}

void SkeletonMarker::ResetColor()
{
  mHasColor = false;

  for (auto const &it : mBodyNodeMarkers) {
    it.second->ResetColor();
  }
}

std::vector<BodyNodeMarkerPtr> SkeletonMarker::bodynode_markers() const
{
  std::vector<BodyNodeMarkerPtr> bodynode_markers;
  bodynode_markers.reserve(mBodyNodeMarkers.size());

  for (auto const &it : mBodyNodeMarkers) {
    bodynode_markers.push_back(it.second);
  }

  return bodynode_markers;
}
