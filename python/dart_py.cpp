#include <pybind11/pybind11.h>
#include <memory>

#include <dart/dynamics/dynamics.hpp>

namespace py = pybind11;

namespace aikido {
namespace python {

void MetaSkeleton(pybind11::module& m)
{
  py::class_<dart::dynamics::MetaSkeleton, std::shared_ptr<dart::dynamics::MetaSkeleton>>(m, "MetaSkeleton")
      .def("getName",
           [](dart::dynamics::MetaSkeleton* self) -> std::string { return self->getName(); });
}

void Skeleton(pybind11::module& m)
{
  py::class_<dart::dynamics::Skeleton, std::shared_ptr<dart::dynamics::Skeleton>>(m, "Skeleton")
      .def("getName",
           [](dart::dynamics::Skeleton* self) -> std::string { return self->getName(); });
}


} // namespace python
} // namespace aikido
