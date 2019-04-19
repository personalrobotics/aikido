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
    const dart::dynamics::BodyNodePtr bodynode,
    aikido::constraint::TestablePtr collisionConstraint)
  : mMarkerServer(markerServer)
  , mInteractiveMarker()
  , mFrameId(frameId)
  , mManipulatorSkeleton(std::move(manipulatorSkeleton))
  , mBodyNode(bodynode)
  , mNeedUpdate(false)
{
  // Create an IK solver with metaSkeleton dofs.
  mInverseKinematics = dart::dynamics::InverseKinematics::create(mBodyNode);

  // Setting invariant properties
  mInteractiveMarker.header.frame_id = mFrameId;
  mInteractiveMarker.name = markerName;
  mInteractiveMarker.pose
      = convertEigenToROSPose(mBodyNode->getWorldTransform());
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
  mMarkerServer->setCallback(mInteractiveMarker.name, boost::bind(&ManipulatorMarker::markerCallback, this, _1));

  // TODO (avk): Optionally create a rviz panel to display and set joints.
}

//==============================================================================
ManipulatorMarker::~ManipulatorMarker()
{
  mMarkerServer->erase(mInteractiveMarker.name);
}

//==============================================================================
void ManipulatorMarker::update()
{
  if (!mNeedUpdate)
    return;

  // Update the target pose for the IK Solver.
  mInverseKinematics->getTarget()->setTransform(mMarkerPose);

  // Solve IK. If successful, update the skeleton.
  bool success = mInverseKinematics->solve(true);
  DART_UNUSED(success);

  mNeedUpdate = false;
}

//==============================================================================
void ManipulatorMarker::markerCallback(InteractiveMarkerFeedbackConstPtr const& feedback)
{
  // Compute the IK of the robot and set state accordingle.
  if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE) 
  {
    auto markerPose = feedback->pose;

    // Extract the marker pose.
    mMarkerPose = Eigen::Translation3d(markerPose.position.x,
                                       markerPose.position.y,
                                       markerPose.position.z) *
                  Eigen::Quaterniond(markerPose.orientation.w,
                                     markerPose.orientation.x,
                                     markerPose.orientation.y,
                                     markerPose.orientation.z);

    mNeedUpdate = true;
  }
}

//==============================================================================
Marker& ManipulatorMarker::getMarker()
{
  // TODO (avk): Is this the marker to return?
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
