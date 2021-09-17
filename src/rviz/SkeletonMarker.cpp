#include "aikido/rviz/SkeletonMarker.hpp"

using aikido::rviz::BodyNodeMarker;
using aikido::rviz::BodyNodeMarkerPtr;
using aikido::rviz::SkeletonMarker;
using dart::dynamics::BodyNode;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::WeakBodyNodePtr;
using dart::dynamics::WeakSkeletonPtr;
using interactive_markers::InteractiveMarkerServer;

namespace aikido {
namespace rviz {

//==============================================================================
SkeletonMarker::SkeletonMarker(
    ResourceServer* resourceServer,
    InteractiveMarkerServer* markerServer,
    const WeakSkeletonPtr& skeleton,
    const std::string& frameId)
  : mSkeleton(skeleton)
  , mResourceServer(resourceServer)
  , mMarkerServer(markerServer)
  , mHasColor(false)
  , mFrameId(frameId)
{
  // Do nothing
}

//==============================================================================
dart::dynamics::SkeletonPtr SkeletonMarker::getSkeleton() const
{
  return mSkeleton.lock();
}

//==============================================================================
bool SkeletonMarker::update()
{
  SkeletonPtr const skeleton = mSkeleton.lock();
  if (!skeleton)
    return false;

  for (BodyNode* const bodyNode : skeleton->getBodyNodes())
  {
    // Lazily create a BodyNodeMarker.
    auto result = mBodyNodeMarkers.insert(
        std::make_pair(bodyNode, BodyNodeMarkerPtr()));
    BodyNodeMarkerPtr& bodyNodeMarker = result.first->second;

    if (result.second)
    {
      bodyNodeMarker = std::make_shared<BodyNodeMarker>(
          mResourceServer, mMarkerServer, WeakBodyNodePtr(bodyNode), mFrameId);

      if (mHasColor)
        bodyNodeMarker->SetColor(mColor);
    }

    // Update the BodyNodeMarker. If update() returns false, then the BodyNode
    // was deleted.
    if (!bodyNodeMarker->update())
      mBodyNodeMarkers.erase(result.first);
  }
  return true;
}

//==============================================================================
BodyNodeMarkerPtr SkeletonMarker::GetBodyNodeMarker(
    const dart::dynamics::BodyNode* bodynode) const
{
  const auto it = mBodyNodeMarkers.find(bodynode);
  if (it != std::end(mBodyNodeMarkers))
    return it->second;
  else
    throw std::runtime_error("There is no marker for this BodyNode.");
}

//==============================================================================
void SkeletonMarker::SetColor(const Eigen::Vector4d& color)
{
  mColor = color;
  mHasColor = true;

  for (const auto& it : mBodyNodeMarkers)
    it.second->SetColor(color);
}

//==============================================================================
void SkeletonMarker::ResetColor()
{
  mHasColor = false;

  for (const auto& it : mBodyNodeMarkers)
    it.second->ResetColor();
}

//==============================================================================
std::vector<BodyNodeMarkerPtr> SkeletonMarker::bodynode_markers() const
{
  std::vector<BodyNodeMarkerPtr> bodynode_markers;
  bodynode_markers.reserve(mBodyNodeMarkers.size());

  for (const auto& it : mBodyNodeMarkers)
    bodynode_markers.push_back(it.second);

  return bodynode_markers;
}

} // namespace rviz
} // namespace aikido
