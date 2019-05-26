#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

// void Observer(py::module& sm);
// void Subject(py::module& sm);
// void Uri(py::module& sm);
// void Composite(py::module& sm);

void aikidopy_common(py::module& m)
{
  auto sm = m.def_submodule("common");

//   Observer(sm);
//   Subject(sm);
//   Uri(sm);
//   Composite(sm);
}

} // namespace python
} // namespace aikido
