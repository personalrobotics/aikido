#ifndef DART_INTERACTIVEMARKER_SHAPE_CONVERSIONS_H_
#define DART_INTERACTIVEMARKER_SHAPE_CONVERSIONS_H_

#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include "ResourceServer.h"

struct aiMesh;

namespace dart {

namespace dynamics {

class Shape;
class BoxShape;
class CylinderShape;
class EllipsoidShape;
class LineSegmentShape;
class MeshShape;
class PlaneShape;
class SoftMeshShape;

} // namespace dynamics

namespace rviz {

geometry_msgs::Point convertEigenToROSPoint(Eigen::Vector3d const &v);
geometry_msgs::Vector3 convertEigenToROSVector3(Eigen::Vector3d const &v);
geometry_msgs::Quaternion convertEigenToROSQuaternion(Eigen::Quaterniond const &v);
geometry_msgs::Pose convertEigenToROSPose(Eigen::Isometry3d const &v);
std_msgs::ColorRGBA convertEigenToROSColorRGBA(Eigen::Vector4d const &v);

bool convertAssimpMeshToROSTriangleList(aiMesh const &mesh,
  std::vector<geometry_msgs::Point> *triangle_list);

bool convertShape(dart::dynamics::Shape const &shape,
                  visualization_msgs::Marker *marker,
                  ResourceServer *resourceManager);
bool convertShape(dart::dynamics::BoxShape const &shape,
                  visualization_msgs::Marker *marker,
                  ResourceServer *resourceManager);
bool convertShape(dart::dynamics::CylinderShape const &shape,
                  visualization_msgs::Marker *marker,
                  ResourceServer *resourceManager);
bool convertShape(dart::dynamics::EllipsoidShape const &shape,
                  visualization_msgs::Marker *marker,
                  ResourceServer *resourceManager);
bool convertShape(dart::dynamics::LineSegmentShape const &shape,
                  visualization_msgs::Marker *marker,
                  ResourceServer *resourceManager);
bool convertShape(dart::dynamics::MeshShape const &shape,
                  visualization_msgs::Marker *marker,
                  ResourceServer *resourceManager);
bool convertShape(dart::dynamics::PlaneShape const &shape,
                  visualization_msgs::Marker *marker,
                  ResourceServer *resourceManager,
                  double width = 100.0);
bool convertShape(dart::dynamics::SoftMeshShape const &shape,
                  visualization_msgs::Marker *marker,
                  ResourceServer *resourceManager);

} // namespace rviz
} // namespace dart

#endif
