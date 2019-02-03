#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

void eigen_geometry(py::module& m);

// DART
void Skeleton(py::module& m);

void MetaSkeleton(py::module& m);

// AIKIDO
void CollisionFree(py::module& m);

void ConcreteManipulator(py::module& m);

void TSR(py::module& m);

void WorldInteractiveMarkerViewer(py::module& m);

void TSRMarker(py::module& m);

void World(py::module& m);

PYBIND11_MODULE(aikidopy, m)
{
  py::module::import("numpy");

  m.doc() = "aikido python bindings";

  eigen_geometry(m);

  CollisionFree(m);

  ConcreteManipulator(m);

  TSR(m);

  WorldInteractiveMarkerViewer(m);

  TSRMarker(m);

  World(m);

  // DART
  MetaSkeleton(m);

  Skeleton(m);
}

} // namespace python
} // namespace aikido
