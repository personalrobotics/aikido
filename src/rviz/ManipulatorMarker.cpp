#include "aikido/rviz/ManipulatorMarker.hpp"
#include "aikido/rviz/shape_conversions.hpp"

using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::InteractiveMarkerFeedback;
using visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using visualization_msgs::Marker;
using interactive_markers::InteractiveMarkerServer;

namespace aikido {
namespace rviz {

//==============================================================================
ManipulatorMarker::ManipulatorMarker(
    interactive_markers::InteractiveMarkerServer* markerServer,
    const std::string& frameId,
    const std::string& markerName,
    dart::dynamics::MetaSkeletonPtr manipulatorSkeleton,
    const dart::dynamics::Frame& frame,
    aikido::constraint::TestablePtr collisionConstraint)
  : mMarkerServer(markerServer)
  , mInteractiveMarker()
  , mFrameId(frameId)
  , mManipulatorSkeleton(std::move(manipulatorSkeleton))
  , mFrame(frame)
  , mNeedUpdate(true)
{
  // Setting invariant properties
  mInteractiveMarker.header.frame_id = mFrameId;
  mInteractiveMarker.name = markerName;
  mInteractiveMarker.pose
      = convertEigenToROSPose(mFrame.getWorldTransform());
  mInteractiveMarker.scale = 0.3;

  InteractiveMarkerControl control;

  // Add translation and rotation about x-axis.
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  mInteractiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  mInteractiveMarker.controls.push_back(control);

  // Add translation and rotation about y-axis.
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  mInteractiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  mInteractiveMarker.controls.push_back(control);

  // Add translation and rotation about z-axis.
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  mInteractiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  mInteractiveMarker.controls.push_back(control);

  mMarkerServer->insert(mInteractiveMarker);
  mMarkerServer->setCallback(mInteractiveMarker.name, boost::bind(&ManipulatorMarker::getMarkerPose, this, _1));

  // TODO (avk): Optionally create a menu for setting DOF values.
  // createMenu();
}

//==============================================================================
ManipulatorMarker::~ManipulatorMarker()
{
  mMarkerServer->erase(mInteractiveMarker.name);
}

//==============================================================================
void ManipulatorMarker::update()
{
  // Update the marker.

  // Insert the marker into the server. TODO (avk): Is this step necessary?
  mMarkerServer->insert(mInteractiveMarker);

  mNeedUpdate = false;
}

//==============================================================================
void ManipulatorMarker::getMarkerPose(InteractiveMarkerFeedbackConstPtr const& feedback)
{
  // Compute the IK of the robot and set state accordingle.
  if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) 
  {
    mMarkerPose = feedback->pose;
    mNeedUpdate = true;
  }
}

//==============================================================================
Marker& ManipulatorMarker::getMarker()
{
  Marker& marker = mInteractiveMarker.controls.front().markers.front();
  return marker;
}

//==============================================================================
const Marker& ManipulatorMarker::getMarker() const
{
  auto& marker = const_cast<ManipulatorMarker*>(this)->getMarker();
  return const_cast<const Marker&>(marker);
}

} // namespace rviz
} // namespace aikido
