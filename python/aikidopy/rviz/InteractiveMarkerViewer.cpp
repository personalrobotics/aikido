#ifdef AIKIDO_HAS_RVIZ

#include <pybind11/pybind11.h>
#include <aikido/rviz.hpp>

namespace py = pybind11;

namespace aikido {
namespace python {

void InteractiveMarkerViewer(py::module& m)
{
    py::class_<aikido::rviz::InteractiveMarkerViewer, std::shared_ptr<aikido::rviz::InteractiveMarkerViewer>>(m, "InteractiveMarkerViewer")
      .def("addTSRMarker",
        [](aikido::rviz::InteractiveMarkerViewer* self,
          std::shared_ptr<aikido::constraint::dart::TSR> tsr)
        -> aikido::rviz::TSRMarkerPtr
        {
          return self->addTSRMarker(*tsr.get());
        });
}

} // namespace python
} // namespace aikido

#endif // AIKIDO_HAS_RVIZ
