#include <aikido/rviz/SimpleFrameMarker.hpp>

using dart::dynamics::WeakSimpleFramePtr;
using dart::dynamics::SimpleFramePtr;
using aikido::rviz::SimpleFrameMarker;
using interactive_markers::InteractiveMarkerServer;

namespace aikido {
namespace rviz {

//==============================================================================
SimpleFrameMarker::SimpleFrameMarker(
    ResourceServer* resourceServer,
    InteractiveMarkerServer* markerServer,
    const WeakSimpleFramePtr& frame,
    const std::string& frameId)
  : mSimpleFrame(frame)
  , mResourceServer(resourceServer)
  , mMarkerServer(markerServer)
  , mHasColor(false)
  , mFrameId(frameId)
{
  // Do nothing
}

//==============================================================================
dart::dynamics::SimpleFramePtr SimpleFrameMarker::getSimpleFrame() const
{
  return mSimpleFrame.lock();
}

////==============================================================================
//bool SimpleFrameMarker::update()
//{
//  SimpleFramePtr const frame = mSimpleFrame.lock();
//  if (!frame)
//    return false;

//    // Whatever is done in the bodynode marker update function
//}

////==============================================================================
//void SimpleFrameMarker::SetColor(const Eigen::Vector4d& color)
//{
//  mColor = color;
//  mHasColor = true;


//  // Replace with whatever the bodynode logic is
//  for (const auto& it : mBodyNodeMarkers)
//    it.second->SetColor(color);
//}

////==============================================================================
//void SimpleFrameMarker::ResetColor()
//{
//  mHasColor = false;

//  // Whatever the logix is for bodynode marker
//  for (const auto& it : mBodyNodeMarkers)
//    it.second->ResetColor();
//}

} // namespace rviz
} // namespace aikido
