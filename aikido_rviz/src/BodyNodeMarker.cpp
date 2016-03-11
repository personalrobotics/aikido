#include <sstream>
#include <aikido/rviz/shape_conversions.hpp>
#include <aikido/rviz/BodyNodeMarker.hpp>

using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::WeakBodyNodePtr;
using dart::dynamics::ConstShapePtr;
using aikido::rviz::BodyNodeMarker;
using interactive_markers::InteractiveMarkerServer;

BodyNodeMarker::BodyNodeMarker(ResourceServer *resourceServer,
                               InteractiveMarkerServer *markerServer,
                               WeakBodyNodePtr const &bodyNodeWeak)
  : mBodyNode(bodyNodeWeak)
  , mResourceServer(resourceServer)
  , mMarkerServer(markerServer)
  , mExists(false)
  , mGeometryDirty(true)
  , mHasColor(false)
  , mColor(0.5, 0.5, 0.5, 1.0)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using visualization_msgs::InteractiveMarkerControl;

  mInteractiveMarker.header.frame_id = "map";
  mInteractiveMarker.scale = 1.;
  mInteractiveMarker.controls.resize(1);
  mVisualControl = &mInteractiveMarker.controls[0];
  mVisualControl->orientation_mode = InteractiveMarkerControl::INHERIT;
  mVisualControl->interaction_mode = InteractiveMarkerControl::BUTTON;
  mVisualControl->always_visible = true;

  // Register callbacks on BodyNode changes.
  BodyNodePtr const bodyNode = mBodyNode.lock();
  if (bodyNode) {
    mName = getName(*bodyNode);

    mOnColShapeAdded = bodyNode->onColShapeAdded.connect(
      std::bind(&BodyNodeMarker::onColShapeAdded, this, _1, _2));
    mOnColShapeRemoved = bodyNode->onColShapeRemoved.connect(
      std::bind(&BodyNodeMarker::onColShapeRemoved, this, _1, _2));
    mOnStructuralChange = bodyNode->onStructuralChange.connect(
      std::bind(&BodyNodeMarker::onStructuralChange, this, _1));
  }
}

BodyNodeMarker::~BodyNodeMarker()
{
  if (mExists) {
    mMarkerServer->erase(mInteractiveMarker.name);
  }
}

bool BodyNodeMarker::update()
{
  BodyNodePtr const bodyNode = mBodyNode.lock();
  if (!bodyNode) {
    return false;
  }

  // Update mInteractiveMarker.name.
  if (mName != mInteractiveMarker.name) {
    if (mExists) {
      mMarkerServer->erase(mInteractiveMarker.name);
      mExists = false;
    }

    updateName(*bodyNode, mName);
  }

  bool const do_create = !mExists || mGeometryDirty;

  // Update mVisualControl->markers.
  if (mGeometryDirty) {
    updateGeometry(*bodyNode);
    mGeometryDirty = false;
  }

  // Update mInteractiveMarker.pose
  updatePose(*bodyNode);

  if (do_create) {
    mMarkerServer->insert(mInteractiveMarker);
    mExists = true;
  } else {
    mMarkerServer->setPose(mInteractiveMarker.name, mInteractiveMarker.pose);
  }
  return true;
}

void BodyNodeMarker::SetColor(Eigen::Vector4d const &color)
{
  mGeometryDirty = mGeometryDirty || mHasColor != true || mColor != color;
  mHasColor = true;
  mColor = color;
}

void BodyNodeMarker::ResetColor()
{
  mGeometryDirty = mGeometryDirty || mHasColor != false;
  mHasColor = false;
}

std::string BodyNodeMarker::getName(BodyNode const &bodyNode)
{
  std::stringstream ss;
  ss << bodyNode.getSkeleton()->getName() << ":"
     << bodyNode.getName();
  return ss.str();
}

void BodyNodeMarker::updateName(BodyNode const &bodyNode,
                                std::string const &newName)
{
  mInteractiveMarker.name = newName;
}

void BodyNodeMarker::updateGeometry(BodyNode const &bodyNode)
{
  using aikido::rviz::convertEigenToROSColorRGBA;
  using visualization_msgs::Marker;

  mVisualControl->markers.clear();
  mVisualControl->markers.reserve(bodyNode.getNumVisualizationShapes());

  for (size_t i = 0; i < bodyNode.getNumVisualizationShapes(); ++i) {
    ConstShapePtr const shape = bodyNode.getVisualizationShape(i);
    if (shape->isHidden())
      continue;

    Marker marker;
    convertShape(*shape, &marker, mResourceServer);

    // Override the color of this BodyNode.
    // TODO: This should be per-ShapeNode, pending the refactor.
    if (mHasColor) {
      marker.color = convertEigenToROSColorRGBA(mColor);
    }

    mVisualControl->markers.push_back(marker);
  }
}

void BodyNodeMarker::updatePose(BodyNode const &bodyNode)
{
  Eigen::Isometry3d const &Tworld_bodynode = bodyNode.getWorldTransform();
  mInteractiveMarker.pose = convertEigenToROSPose(Tworld_bodynode);
}

void BodyNodeMarker::onColShapeAdded(BodyNode const *bodyNode,
                                     ConstShapePtr shape)
{
  mGeometryDirty = true;
}

void BodyNodeMarker::onColShapeRemoved(BodyNode const *bodyNode,
                                       ConstShapePtr shape)
{
  mGeometryDirty = true;
}

void BodyNodeMarker::onStructuralChange(BodyNode const *bodyNode)
{
  mName = getName(*bodyNode);
}
