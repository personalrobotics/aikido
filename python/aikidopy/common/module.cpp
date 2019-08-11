#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

void aikidopy_common(py::module& m)
{
  auto sm = m.def_submodule("common");
}

} // namespace python
} // namespace aikido
