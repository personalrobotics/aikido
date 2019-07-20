#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

void World(py::module& sm);

void aikidopy_planner(py::module& m)
{
  auto sm = m.def_submodule("planner");

  World(sm);
}

} // namespace python
} // namespace aikido
