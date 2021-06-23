#ifdef AIKIDO_HAS_RVIZ

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

void TSRMarker(py::module& sm);
void InteractiveMarkerViewer(py::module& sm);

void aikidopy_rviz(py::module& m)
{
  auto sm = m.def_submodule("rviz");

  TSRMarker(sm);
  InteractiveMarkerViewer(sm);
}

} // namespace python
} // namespace aikido

#endif // AIKIDO_HAS_RVIZ
