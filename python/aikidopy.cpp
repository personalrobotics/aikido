#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

void eigen_geometry(pybind11::module& m);

void CollisionFree(pybind11::module& m);

void MetaSkeleton(pybind11::module& m);

void TSR(pybind11::module& m);

PYBIND11_MODULE(aikidopy, m)
{
  py::module::import("numpy");

  m.doc() = "aikido python bindings";

  eigen_geometry(m);

  TSR(m);

  CollisionFree(m);

  // DART
  MetaSkeleton(m);
}

} // namespace python
} // namespace aikido
