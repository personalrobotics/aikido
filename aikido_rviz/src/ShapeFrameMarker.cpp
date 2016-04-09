#include <dart/dynamics/dynamics.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <aikido/rviz/ResourceServer.hpp>
#include <aikido/rviz/ShapeFrameMarker.hpp>
#include <aikido/rviz/shape_conversions.hpp>

using interactive_markers::InteractiveMarkerServer;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::Marker;
using dart::dynamics::CollisionAddon;
using dart::dynamics::VisualAddon;
using dart::dynamics::ShapeFrame;
using dart::dynamics::ConstShapePtr;
using aikido::rviz::ShapeFrameMarker;

namespace {

static const Eigen::Vector4d COLLISION_COLOR(1., 0., 0., 0.5);

} // namespace

ShapeFrameMarker::ShapeFrameMarker(
      ResourceServer *resourceServer,
      InteractiveMarkerServer *markerServer,
      const std::string &name,
      ShapeFrame const *shapeFrame)
  : mShapeFrame(shapeFrame)
  , mResourceServer(resourceServer)
  , mMarkerServer(markerServer)
  , mExists(false)
  , mForceUpdate(true)
  , mVersion()
  , mShowVisual(true)
  , mShowCollision(false)
  , mColor()
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  mInteractiveMarker.header.frame_id = "map";
  mInteractiveMarker.scale = 1.;
  mInteractiveMarker.controls.resize(1);
  mInteractiveMarker.name = name;
  mVisualControl = &mInteractiveMarker.controls.front();
  mVisualControl->orientation_mode = InteractiveMarkerControl::INHERIT;
  mVisualControl->interaction_mode = InteractiveMarkerControl::BUTTON;
  mVisualControl->always_visible = true;
}

ShapeFrameMarker::~ShapeFrameMarker()
{
  if (mExists) {
    mMarkerServer->erase(mInteractiveMarker.name);
  }
}

void ShapeFrameMarker::SetColor(Eigen::Vector4d const &color)
{
  mForceUpdate = !(mColor && color == *mColor);
  mColor.reset(color);
}

void ShapeFrameMarker::ResetColor()
{
  mForceUpdate = !!mColor;
  mColor.reset();
}

bool ShapeFrameMarker::update()
{
  mInteractiveMarker.pose = convertEigenToROSPose(
    mShapeFrame->getWorldTransform());

  // Incrementally update the pose if nothing else have changed.
  const size_t newVersion = mShapeFrame->getVersion();
  const bool do_update = mForceUpdate || newVersion != mVersion;

  if (!do_update) {
    if (mExists) {
      mMarkerServer->setPose(mInteractiveMarker.name, mInteractiveMarker.pose);
    }

    return mExists;
  }

  mForceUpdate = false;
  mVersion = newVersion;

  // Otherwise, create a new marker.
  const ConstShapePtr shape = mShapeFrame->getShape();
  const VisualAddon *visualAddon = mShapeFrame->getVisualAddon();
  const CollisionAddon *collisionAddon = mShapeFrame->getCollisionAddon();

  const bool showVisual
    = mShowVisual && visualAddon && !visualAddon->isHidden();
  const bool showCollision
    = mShowCollision && collisionAddon && collisionAddon->isCollidable();
  
  if (showVisual || showCollision) {
    mVisualControl->markers.resize(1);
    Marker& marker = mVisualControl->markers.front();

    if (!convertShape(*shape, &marker, mResourceServer)) {
      if (mExists) {
        mMarkerServer->erase(mInteractiveMarker.name);
        mExists = false;
      }

      dtwarn << "[ShapeMarkerFrame::update] Failed converting shape of type "
             << shape->getShapeType()
             << " to a visualization_msgs/Marker ROS message.\n";
      return false;
    }

    if (mColor) {
      marker.color = convertEigenToROSColorRGBA(*mColor);
    } else if (showVisual) {
      marker.color = convertEigenToROSColorRGBA(visualAddon->getRGBA());
    } else if (showCollision) {
      marker.color = convertEigenToROSColorRGBA(COLLISION_COLOR);
    } else {
      assert(false && "This should never happen.");
    }
  }
  else
  {
    mVisualControl->markers.clear();
  }

  mMarkerServer->insert(mInteractiveMarker);
  mExists = true;
  return true;
}
