#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

// void TSRMarker(py::module& sm);
// void WorldInteractiveMarkerViewer(py::module& sm);

void aikidopy_statespace(py::module& m)
{
  auto sm = m.def_submodule("statespace");

  // TSRMarker(sm);
  // WorldInteractiveMarkerViewer(sm);
}

} // namespace python
} // namespace aikido
