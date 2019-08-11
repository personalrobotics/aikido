#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

void aikidopy_statespace(py::module& m)
{
  auto sm = m.def_submodule("statespace");
}

} // namespace python
} // namespace aikido
