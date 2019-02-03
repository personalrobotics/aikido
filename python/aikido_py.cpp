#include <pybind11/pybind11.h>
#include <memory>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/robot/ConcreteManipulator.hpp>
#include <aikido/constraint/dart/TSR.hpp>

namespace py = pybind11;

namespace aikido {
namespace python {


void CollisionFree(pybind11::module& m)
{
  py::class_<aikido::constraint::dart::CollisionFree, std::shared_ptr<aikido::constraint::dart::CollisionFree>>(m, "CollisionFree");
}

void TSR(pybind11::module& m)
{
  py::class_<aikido::constraint::dart::TSR, std::shared_ptr<aikido::constraint::dart::TSR>>(m, "TSR");
}

} // namespace python
} // namespace aikido
