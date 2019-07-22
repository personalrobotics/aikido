#ifdef AIKIDO_HAS_RVIZ

#include <pybind11/pybind11.h>
#include <aikido/rviz.hpp>

namespace py = pybind11;

namespace aikido {
namespace python {

void TSRMarker(py::module& m)
{
  py::class_<aikido::rviz::TSRMarker, std::shared_ptr<aikido::rviz::TSRMarker>>(m, "TSRMarker");
}

} // namespace python
} // namespace aikido

#endif // AIKIDO_HAS_RVIZ
