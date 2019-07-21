#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

void ConcreteManipulator(py::module& sm);

void aikidopy_robot(py::module& m)
{
  auto sm = m.def_submodule("robot");

  ConcreteManipulator(sm);
}

} // namespace python
} // namespace aikido
