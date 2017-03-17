#include <boost/format.hpp>
#include <aikido/rviz/shape_conversions.hpp>
#include <aikido/rviz/FrameMarker.hpp>

using boost::format;
using boost::str;

namespace aikido {
namespace rviz {

static geometry_msgs::Point makePoint(double x, double y, double z)
{
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

static std_msgs::ColorRGBA makeColorRGBA(double r, double g, double b,
                                         double a = 1.)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

static void CreateAxis(
  Eigen::Vector3d const &axis, double length, double thickness,
  visualization_msgs::Marker *marker)
{
  // Orient the desired axis along the z-axis, which is the axis of the
  // cylinder in RViz.
  Eigen::Quaterniond quat;
  quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), axis);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.rotate(quat);
  pose.pretranslate(0.5 * length * axis);

  marker->type = visualization_msgs::Marker::CYLINDER;
  marker->scale.x = thickness;
  marker->scale.y = thickness;
  marker->scale.z = length;
  marker->pose = convertEigenToROSPose(pose);
}

FrameMarker::FrameMarker(
      interactive_markers::InteractiveMarkerServer *markerServer,
      dart::dynamics::Frame *frame,
      double length, double thickness, double alpha)
  : mMarkerServer(markerServer)
  , mFrame(frame)
{
  using visualization_msgs::InteractiveMarkerControl;

  static geometry_msgs::Point const origin = makePoint(0, 0, 0);

  mInteractiveMarker.header.frame_id = "map";
  mInteractiveMarker.name = str(format("Frame[%s]") % frame->getName());
  mInteractiveMarker.pose.orientation.w = 1;
  mInteractiveMarker.scale = 1;

  mInteractiveMarker.controls.resize(1);
  InteractiveMarkerControl &control = mInteractiveMarker.controls.front();
  control.orientation.w = 1;
  control.orientation_mode = InteractiveMarkerControl::INHERIT;
  control.interaction_mode = InteractiveMarkerControl::NONE;
  control.always_visible = true;

  control.markers.resize(3);
  CreateAxis(Eigen::Vector3d::UnitX(), length, thickness, &control.markers[0]);
  CreateAxis(Eigen::Vector3d::UnitY(), length, thickness, &control.markers[1]);
  CreateAxis(Eigen::Vector3d::UnitZ(), length, thickness, &control.markers[2]);
  control.markers[0].color = makeColorRGBA(1, 0, 0, alpha);
  control.markers[1].color = makeColorRGBA(0, 1, 0, alpha);
  control.markers[2].color = makeColorRGBA(0, 0, 3, alpha);

  mMarkerServer->insert(mInteractiveMarker);
  update();
}

FrameMarker::~FrameMarker()
{
  mMarkerServer->erase(mInteractiveMarker.name);
}

void FrameMarker::update()
{
  mMarkerServer->setPose(mInteractiveMarker.name,
    convertEigenToROSPose(mFrame->getTransform()));
}

} // namespace rviz
} // namespace aikido
