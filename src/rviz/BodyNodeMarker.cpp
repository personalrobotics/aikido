#include <sstream>
#include <unordered_set>
#include <aikido/rviz/shape_conversions.hpp>
#include <aikido/rviz/BodyNodeMarker.hpp>

using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::ConstBodyNodePtr;
using dart::dynamics::WeakBodyNodePtr;
using dart::dynamics::ShapeFrame;
using dart::dynamics::ShapeNode;
using aikido::rviz::BodyNodeMarker;
using interactive_markers::InteractiveMarkerServer;

BodyNodeMarker::BodyNodeMarker(ResourceServer *resourceServer,
                               InteractiveMarkerServer *markerServer,
                               WeakBodyNodePtr const &bodyNodeWeak,
                               const std::string &frameId)
  : mBodyNode(bodyNodeWeak)
  , mResourceServer(resourceServer)
  , mMarkerServer(markerServer)
  , mFrameId(frameId)
{
  // Register callbacks on BodyNode changes.
  BodyNodePtr const bodyNode = mBodyNode.lock();
  if (bodyNode) {
    mName = getName(*bodyNode);
  }

  // TODO: Rename the marker if the BodyNode changes name.
}

bool BodyNodeMarker::update()
{
  ConstBodyNodePtr const bodyNode = mBodyNode.lock();
  if (!bodyNode) {
    return false;
  }

  // Match the ShapeNodes attached to the BodyNode against the list of
  // ShapeFrameMarkers that already exist.
  const std::vector<const ShapeNode *> currShapeNodes
    = bodyNode->getShapeNodes();
  std::set<const ShapeNode *> pendingShapeNodes(
    std::begin(currShapeNodes), std::end(currShapeNodes));

  auto mapIt = std::begin(mShapeFrameMarkers);
  while (mapIt != std::end(mShapeFrameMarkers)) {
    const auto setIt = pendingShapeNodes.find(mapIt->first);

    // Shape node already exists. Don't try to create a new ShapeFrameMarker.
    if (setIt != std::end(pendingShapeNodes)) {
      ++mapIt;
      pendingShapeNodes.erase(setIt);
    }
    // ShapeNode does not exist Delete our existing ShapeFrameMarker.
    else {
      mapIt = mShapeFrameMarkers.erase(mapIt);
    }
  }

  // Create any new ShapeFrameMarkers that are necessary.
  for (const ShapeNode *shapeNode : currShapeNodes) {
    // TODO: Placeholder for an actual name.
    // TODO: Set the correct default color on this.
    std::stringstream shapeNodeName;
    shapeNodeName << "ShapeNode[" << shapeNode << "]";

    mShapeFrameMarkers.emplace(shapeNode, std::unique_ptr<ShapeFrameMarker>(
      new ShapeFrameMarker(mResourceServer, mMarkerServer, shapeNodeName.str(),
        shapeNode, mFrameId)));
  }

  // Update all of the ShapeFrameMarkers.
  bool does_exist = false;

  for (const auto &it : mShapeFrameMarkers) {
    if (it.second->update())
      does_exist = true;
  }

  return does_exist;
}

void BodyNodeMarker::SetColor(Eigen::Vector4d const &color)
{
  for (const auto &it : mShapeFrameMarkers)
    it.second->SetColor(color);
}

void BodyNodeMarker::ResetColor()
{
  for (const auto &it : mShapeFrameMarkers)
    it.second->ResetColor();
}

std::string BodyNodeMarker::getName(BodyNode const &bodyNode)
{
  std::stringstream ss;
  ss << bodyNode.getSkeleton()->getName() << ":"
     << bodyNode.getName();
  return ss.str();
}
