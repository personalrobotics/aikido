#include <pybind11/pybind11.h>
#include <dart/utils/urdf/urdf.hpp>
#include <aikido/io.hpp>
#include <aikido/planner.hpp>

namespace py = pybind11;

namespace aikido {
namespace python {

Eigen::Isometry3d vectorToIsometry(std::vector<double>& poseVector)
{
  double* ptr = &poseVector[0];
  Eigen::Map<Eigen::VectorXd> p(ptr, 7);

  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  pose.translation() = p.head(3);
  Eigen::Quaterniond q(p[3], p[4], p[5], p[6]);
  pose.linear() = Eigen::Matrix3d(q);
  return pose;
}

void World(py::module& m)
{
  py::class_<aikido::planner::World, std::shared_ptr<aikido::planner::World>>(m, "World")
      .def("addBodyFromURDF",
           [](aikido::planner::World* self,
           const std::string& uri,
           std::vector<double> objectPose) // x, y, z, qw, qx, qy, qz)
      -> std::shared_ptr<::dart::dynamics::Skeleton>
  {
    auto transform = vectorToIsometry(objectPose);

    dart::utils::DartLoader urdfLoader;
    const auto resourceRetriever
        = std::make_shared<aikido::io::CatkinResourceRetriever>();
    const auto skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

    if (!skeleton)
      throw std::runtime_error("unable to load '" + uri + "'");

    dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
        ->setTransform(transform);

    self->addSkeleton(skeleton);
    return skeleton;
  }
  );
}

} // namespace python
} // namespace aikido
