#include "aikido/rviz/SimpleFrameMarker.hpp"
#include "aikido/rviz/shape_conversions.hpp"

using dart::dynamics::SimpleFrame;
using dart::dynamics::SimpleFramePtr;
using aikido::rviz::SimpleFrameMarker;
using interactive_markers::InteractiveMarkerServer;

namespace aikido {
namespace rviz {

//==============================================================================
//SimpleFrameMarker(
//  interactive_markers::InteractiveMarkerServer* markerServer,
//  const std::string& frameId,
//  const std::string& markerName,
//  const dart::dynamics::SimpleFramePtr frame,
//  const Eigen::Vector4d& rgba = Eigen::Vector4d::Constant(0.75))
//  : mMarkerServer(markerServer)
//  , mInteractiveMarker()
//  , mFrameId(frameId)
//  , mSimpleFrame(frame)
//  , mNeedUpdate(true)
//{
//  using visualization_msgs::InteractiveMarkerControl;
//  using visualization_msgs::Marker;

//  // Setting invariant properties
//  mInteractiveMarker.header.frame_id = mFrameId;
//  mInteractiveMarker.name = markerName;
//  mInteractiveMarker.pose.orientation.w = 1;
//  mInteractiveMarker.scale = 1;

//  mInteractiveMarker.controls.resize(1);
//  InteractiveMarkerControl& control = mInteractiveMarker.controls.front();
//  control.orientation.w = 1;
//  control.orientation_mode = InteractiveMarkerControl::INHERIT;
//  control.interaction_mode = InteractiveMarkerControl::NONE;
//  control.always_visible = true;
//  control.markers.resize(1);

//  auto& marker = getMarker();
//  marker.type = Marker::CUBE;
//  marker.pose.orientation.w = 1.0;

//  // Setting variant properties
//  setRGBA(rgba);
//}

////==============================================================================
//SimpleFrameMarker::~SimpleFrameMarker()
//{
//  mMarkerServer->erase(mInteractiveMarker.name);
//}

////==============================================================================
//dart::dynamics::SimpleFramePtr SimpleFrameMarker::getSimpleFrame() const
//{
//  return mSimpleFrame;
//}

////==============================================================================
//void SimpleFrameMarker::setColor(const Eigen::Vector3d& rgb)
//{
//  auto& marker = getMarker();
//  marker.color.r = rgb[0];
//  marker.color.g = rgb[1];
//  marker.color.b = rgb[2];

//  mNeedUpdate = true;
//}

////==============================================================================
//Eigen::Vector3d SimpleFrameMarker::getColor() const
//{
//  const auto& marker = getMarker();
//  return convertROSColorRGBAToEigen(marker.color).head<3>();
//}

////==============================================================================
//void SimpleFrameMarker::setRGBA(const Eigen::Vector4d& rgba)
//{
//  auto& marker = getMarker();
//  marker.color = convertEigenToROSColorRGBA(rgba);

//  mNeedUpdate = true;
//}

////==============================================================================
//Eigen::Vector4d SimpleFrameMarker::getRBGA() const
//{
//  const auto& marker = getMarker();
//  return convertROSColorRGBAToEigen(marker.color);
//}

////==============================================================================
//void SimpleFrameMarker::update()
//{
//  if (!mNeedUpdate)
//    return;

//  mMarkerServer->insert(mInteractiveMarker);

//  mNeedUpdate = false;
//}

////==============================================================================
//visualization_msgs::Marker& SimpleFrameMarker::getMarker()
//{
//  using visualization_msgs::Marker;
//  Marker& marker = mInteractiveMarker.controls.front().markers.front();
//  return marker;
//}

////==============================================================================
//const visualization_msgs::Marker& SimpleFrameMarker::getMarker() const
//{
//  auto& marker = const_cast<SimpleFrameMarker*>(this)->getMarker();
//  return const_cast<const visualization_msgs::Marker&>(marker);
//}

} // namespace rviz
} // namespace aikido
