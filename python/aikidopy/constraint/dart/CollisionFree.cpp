#include <pybind11/pybind11.h>
#include <aikido/constraint.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>

namespace py = pybind11;

namespace aikido {
namespace python {

void CollisionFree(py::module& m)
{
  py::class_<aikido::constraint::dart::CollisionFree, std::shared_ptr<aikido::constraint::dart::CollisionFree>>(m, "CollisionFree");
}

} // namespace python
} // namespace aikido
