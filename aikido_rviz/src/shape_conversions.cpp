#include <assimp/cexport.h>
#include <boost/filesystem.hpp>
#include <Eigen/StdVector>
#include <dart/dynamics/dynamics.h>
#include <aikido/rviz/shape_conversions.hpp>
#include <aikido/rviz/ResourceServer.hpp>

using dart::dynamics::Shape;
using dart::dynamics::BoxShape;
using dart::dynamics::CylinderShape;
using dart::dynamics::EllipsoidShape;
using dart::dynamics::LineSegmentShape;
using dart::dynamics::MeshShape;
using dart::dynamics::PlaneShape;
using dart::dynamics::SoftMeshShape;
using visualization_msgs::Marker;

namespace aikido {
namespace rviz {

geometry_msgs::Point convertEigenToROSPoint(Eigen::Vector3d const &v)
{
  geometry_msgs::Point v_ros;
  v_ros.x = v[0];
  v_ros.y = v[1];
  v_ros.z = v[2];
  return v_ros;
}

geometry_msgs::Vector3 convertEigenToROSVector3(Eigen::Vector3d const &v)
{
  geometry_msgs::Vector3 v_ros;
  v_ros.x = v[0];
  v_ros.y = v[1];
  v_ros.z = v[2];
  return v_ros;
}

std_msgs::ColorRGBA convertEigenToROSColorRGBA(Eigen::Vector4d const &v)
{
  std_msgs::ColorRGBA v_ros;
  v_ros.r = v[0];
  v_ros.g = v[1];
  v_ros.b = v[2];
  v_ros.a = v[3];
  return v_ros;
}

geometry_msgs::Quaternion convertEigenToROSQuaternion(Eigen::Quaterniond const &v)
{
  geometry_msgs::Quaternion v_ros;
  v_ros.w = v.w();
  v_ros.x = v.x();
  v_ros.y = v.y();
  v_ros.z = v.z();
  return v_ros;
}

geometry_msgs::Pose convertEigenToROSPose(Eigen::Isometry3d const &v)
{
  Eigen::Quaterniond const quaternion(v.rotation());

  geometry_msgs::Pose v_ros;
  v_ros.orientation = convertEigenToROSQuaternion(quaternion);
  v_ros.position = convertEigenToROSPoint(v.translation());
  return v_ros;
}

bool convertAssimpMeshToROSTriangleList(
  aiMesh const &mesh, std::vector<geometry_msgs::Point> *triangle_list)
{
  triangle_list->reserve(triangle_list->size() + 3 * mesh.mNumFaces);

  for (unsigned int iface = 0; iface < mesh.mNumFaces; ++iface) {
    aiFace const &face = mesh.mFaces[iface];

    if (face.mNumIndices != 3) {
      return false;
    }

    for (unsigned int i = 0; i < 3; ++i) {
      unsigned int ivertex = face.mIndices[i];
      aiVector3D const &vertex = mesh.mVertices[ivertex];

      geometry_msgs::Point ros_vertex;
      ros_vertex.x = vertex.x;
      ros_vertex.y = vertex.y;
      ros_vertex.z = vertex.z;

      triangle_list->push_back(ros_vertex);
    }
  }
  return false;
}

bool convertShape(BoxShape const &shape, Marker *marker,
                  ResourceServer *resourceManager)
{
  marker->type = Marker::CUBE;
  marker->pose = convertEigenToROSPose(shape.getLocalTransform());
  marker->scale = convertEigenToROSVector3(shape.getSize());
  marker->color = convertEigenToROSColorRGBA(shape.getRGBA());
  return true;
}

bool convertShape(CylinderShape const &shape, Marker *marker,
                  ResourceServer *resourceManager)
{
  marker->type = Marker::CYLINDER;
  marker->pose = convertEigenToROSPose(shape.getLocalTransform());
  marker->scale.x = 2. * shape.getRadius();
  marker->scale.y = 2. * shape.getRadius();
  marker->scale.z = shape.getHeight();
  marker->color = convertEigenToROSColorRGBA(shape.getRGBA());
  return true;
}

bool convertShape(EllipsoidShape const &shape, Marker *marker,
                  ResourceServer *resourceManager)
{
  marker->type = Marker::SPHERE;
  marker->pose = convertEigenToROSPose(shape.getLocalTransform());
  marker->scale = convertEigenToROSVector3(shape.getSize());
  marker->color = convertEigenToROSColorRGBA(shape.getRGBA());
  return true;
}

bool convertShape(LineSegmentShape const &shape, Marker *marker,
                  ResourceServer *resourceManager)
{
  std::vector<Eigen::Vector3d> const &vertices = shape.getVertices();
  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> >
    const &connections = shape.getConnections();

  marker->type = Marker::LINE_STRIP;
  marker->pose = convertEigenToROSPose(shape.getLocalTransform());
  marker->color = convertEigenToROSColorRGBA(shape.getRGBA());
  marker->scale.x = shape.getThickness();
  marker->points.reserve(2 * connections.size());

  for (Eigen::Vector2i const &connection : connections) {
    size_t const &i1 = connection[0];
    size_t const &i2 = connection[1];

    if (i1 >= connections.size() || i2 >= connections.size()) {
      return false;
    }

    Eigen::Vector3d const &p1 = vertices[i1];
    Eigen::Vector3d const &p2 = vertices[i2];
    marker->points.push_back(convertEigenToROSPoint(p1));
    marker->points.push_back(convertEigenToROSPoint(p2));
  }
  return true;
}

bool convertShape(MeshShape const &shape, Marker *marker,
                  ResourceServer *resourceManager)
{
  marker->pose = convertEigenToROSPose(shape.getLocalTransform());
  marker->scale = convertEigenToROSVector3(shape.getScale());

  aiScene const *scene = shape.getMesh();
  std::string const &meshUri = shape.getMeshUri();
  if (!meshUri.empty()) {
    marker->type = Marker::MESH_RESOURCE;
    marker->mesh_resource = meshUri;
    marker->mesh_use_embedded_materials = true;
    return true;
  }

  // Fall back on publishing the mesh as a TRIANGLE_LIST.
  if (scene) {
    marker->type = Marker::TRIANGLE_LIST;
    marker->color = convertEigenToROSColorRGBA(shape.getRGBA());

    for (unsigned int imesh = 0; imesh < scene->mNumMeshes; ++imesh) {
      if (!convertAssimpMeshToROSTriangleList(*scene->mMeshes[imesh],
                                              &marker->points)) {
        return false;
      }
    }
    return true;
  }

  return false; // Everything failed!
}

bool convertShape(PlaneShape const &shape, Marker *marker,
                  ResourceServer *resourceManager, double width)
{
  static Eigen::Matrix<double, 3, 6> const points = (
    Eigen::Matrix<double, 6, 3>() <<
      -0.5, -0.5, 0,
       0.5,  0.5, 0,
      -0.5,  0.5, 0,
      -0.5, -0.5, 0,
       0.5, -0.5, 0,
       0.5,  0.5, 0
  ).finished().transpose();

  marker->type = Marker::TRIANGLE_LIST;
  marker->color = convertEigenToROSColorRGBA(shape.getRGBA());

  Eigen::Vector3d const &normal = shape.getNormal();
  double const offset = shape.getOffset();

  // Constructing a frame on the plane at the point closest to the origin.
  Eigen::Quaterniond quat;
  quat.setFromTwoVectors(Eigen::Vector3d::UnitZ(), normal);

  Eigen::Affine3d frame(quat);
  frame.pretranslate(normal * offset);
  frame.scale(width);

  Eigen::MatrixXd const transformedPoints = frame * points;
  marker->points.resize(transformedPoints.cols());

  for (size_t i = 0; i < transformedPoints.cols(); ++i) {
    marker->points[i] = convertEigenToROSPoint(transformedPoints.col(i));
  }
  return true;
}

bool convertShape(SoftMeshShape const &shape, Marker *marker,
                  ResourceServer *resourceManager)
{
  marker->type = Marker::TRIANGLE_LIST;
  marker->pose = convertEigenToROSPose(shape.getLocalTransform());
  marker->color = convertEigenToROSColorRGBA(shape.getRGBA());

  aiMesh const *mesh = shape.getAssimpMesh();
  if (mesh) {
    return convertAssimpMeshToROSTriangleList(*mesh, &marker->points);
  } else {
    return false;
  }
}

bool convertShape(Shape const &shape, Marker *marker, ResourceServer *rm)
{
  switch (shape.getShapeType()) {
  case Shape::BOX:
    return convertShape(dynamic_cast<BoxShape const &>(shape), marker, rm);

  case Shape::ELLIPSOID:
    return convertShape(dynamic_cast<EllipsoidShape const &>(shape), marker, rm);

  case Shape::CYLINDER:
    return convertShape(dynamic_cast<CylinderShape const &>(shape), marker, rm);

  case Shape::LINE_SEGMENT:
    return convertShape(dynamic_cast<LineSegmentShape const &>(shape), marker, rm);

  case Shape::MESH:
    return convertShape(dynamic_cast<MeshShape const &>(shape), marker, rm);

  case Shape::PLANE:
    return convertShape(dynamic_cast<PlaneShape const &>(shape), marker, rm);

  case Shape::SOFT_MESH:
    return convertShape(dynamic_cast<SoftMeshShape const &>(shape), marker, rm);

  default:
    return false;
  }
}

} // namespace rviz
} // namespace aikido
