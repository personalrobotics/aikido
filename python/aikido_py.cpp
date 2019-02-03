#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <memory>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/robot/ConcreteManipulator.hpp>
#include <aikido/constraint/dart/TSR.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/rviz/TSRMarker.hpp>
#include <aikido/planner/World.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>


namespace py = pybind11;

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

namespace aikido {
namespace python {


void CollisionFree(pybind11::module& m)
{
  py::class_<aikido::constraint::dart::CollisionFree, std::shared_ptr<aikido::constraint::dart::CollisionFree>>(m, "CollisionFree");
}

void TSR(pybind11::module& m)
{
  py::class_<aikido::constraint::dart::TSR, std::shared_ptr<aikido::constraint::dart::TSR>>(m, "TSR")
  .def("setPose",
    [](aikido::constraint::dart::TSR* self,
          std::vector<double> pose) // x, y, z, qw, qx, qy, qz)
    {
      self->mT0_w = vectorToIsometry(pose);
    })
  .def("multiplyToEndEffectorTransform",
    [](aikido::constraint::dart::TSR* self,
        Eigen::Isometry3d endEffectorTransform)
    {
      self->mTw_e.matrix() *= endEffectorTransform.matrix();
    })
  .def("getPose",
    [](aikido::constraint::dart::TSR* self)
    -> Eigen::Isometry3d
    {
      return self->mT0_w;
    })
  .def("getEndEffectorTransform",
    [](aikido::constraint::dart::TSR* self)
    -> Eigen::Isometry3d
    {
      return self->mTw_e;
    });
}

void ConcreteManipulator(pybind11::module& m)
{
  py::class_<aikido::robot::ConcreteManipulator, std::shared_ptr<aikido::robot::ConcreteManipulator>>(m, "ConcreteManipulator")
      .def("getName",
           [](aikido::robot::ConcreteManipulator* self) -> std::string { return self->getName(); });
}

void WorldInteractiveMarkerViewer(pybind11::module& m)
{
  py::class_<aikido::rviz::WorldInteractiveMarkerViewer, std::shared_ptr<aikido::rviz::WorldInteractiveMarkerViewer>>(m, "WorldInteractiveMarkerViewer")
      .def("addTSRMarker",
        [](aikido::rviz::WorldInteractiveMarkerViewer* self,
          std::shared_ptr<aikido::constraint::dart::TSR> tsr)
        -> aikido::rviz::TSRMarkerPtr
        {
          return self->addTSRMarker(*tsr.get());
        });

}

void TSRMarker(pybind11::module& m)
{
  py::class_<aikido::rviz::TSRMarker, std::shared_ptr<aikido::rviz::TSRMarker>>(m, "TSRMarker");
}


void World(pybind11::module& m)
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
