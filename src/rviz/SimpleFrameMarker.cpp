#include <aikido/rviz/SimpleFrameMarker.hpp>

using dart::dynamics::SimpleFrame;
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
  , mFrameId(frameId)
  , mHasColor(false)
{
  // Register callbacks on frame changes.
  SimpleFramePtr const simpleFrame = mSimpleFrame.lock();
  if (simpleFrame)
    mName = getName(*simpleFrame);
}

//==============================================================================
dart::dynamics::SimpleFramePtr SimpleFrameMarker::getSimpleFrame() const
{
  return mSimpleFrame.lock();
}

//==============================================================================
bool SimpleFrameMarker::update()
{
  SimpleFramePtr const frame = mSimpleFrame.lock();
  if (!frame)
    return false;

  return true;
}

//==============================================================================
void SimpleFrameMarker::SetColor(const Eigen::Vector4d& color)
{
  mColor = color;
  mHasColor = true;

  mShapeFrameMarker->SetColor(color);
}

//==============================================================================
void SimpleFrameMarker::ResetColor()
{
  mHasColor = false;
  mShapeFrameMarker->ResetColor();
}

//==============================================================================
std::string SimpleFrameMarker::getName(const dart::dynamics::SimpleFrame& frame) const
{
  std::stringstream ss;
  ss << frame.getName();
  return ss.str();
}

} // namespace rviz
} // namespace aikido
