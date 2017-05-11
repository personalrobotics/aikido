#ifndef AIKIDO_RVIZ_SHAPE_CONVERSIONS_HPP_
#define AIKIDO_RVIZ_SHAPE_CONVERSIONS_HPP_

#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include "ResourceServer.hpp"

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
} // namespace dart

namespace aikido {
namespace rviz {

geometry_msgs::Point convertEigenToROSPoint(const Eigen::Vector3d& v);
geometry_msgs::Vector3 convertEigenToROSVector3(const Eigen::Vector3d& v);
geometry_msgs::Quaternion convertEigenToROSQuaternion(
    const Eigen::Quaterniond& v);
geometry_msgs::Pose convertEigenToROSPose(const Eigen::Isometry3d& v);
std_msgs::ColorRGBA convertEigenToROSColorRGBA(const Eigen::Vector4d& v);

bool convertAssimpMeshToROSTriangleList(
    const aiMesh& mesh, std::vector<geometry_msgs::Point>* triangle_list);

bool convertShape(
    const dart::dynamics::Shape& shape,
    visualization_msgs::Marker* marker,
    ResourceServer* resourceManager);
bool convertShape(
    const dart::dynamics::BoxShape& shape,
    visualization_msgs::Marker* marker,
    ResourceServer* resourceManager);
bool convertShape(
    const dart::dynamics::CylinderShape& shape,
    visualization_msgs::Marker* marker,
    ResourceServer* resourceManager);
bool convertShape(
    const dart::dynamics::EllipsoidShape& shape,
    visualization_msgs::Marker* marker,
    ResourceServer* resourceManager);
bool convertShape(
    const dart::dynamics::LineSegmentShape& shape,
    visualization_msgs::Marker* marker,
    ResourceServer* resourceManager);
bool convertShape(
    const dart::dynamics::MeshShape& shape,
    visualization_msgs::Marker* marker,
    ResourceServer* resourceManager);
bool convertShape(
    const dart::dynamics::PlaneShape& shape,
    visualization_msgs::Marker* marker,
    ResourceServer* resourceManager,
    double width = 100.0);
bool convertShape(
    const dart::dynamics::SoftMeshShape& shape,
    visualization_msgs::Marker* marker,
    ResourceServer* resourceManager);

} // namespace rviz
} // namespace aikido

#endif
