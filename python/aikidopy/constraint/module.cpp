#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

void aikidopy_constraint_dart(py::module& sm);

void aikidopy_constraint(py::module& m)
{
  auto sm = m.def_submodule("constraint");

  aikidopy_constraint_dart(sm);
}

} // namespace python
} // namespace aikido
