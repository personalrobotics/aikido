#include <pybind11/pybind11.h>
#include <aikido/robot.hpp>

namespace py = pybind11;

namespace aikido {
namespace python {

void Robot(py::module& m)
{
  py::class_<aikido::robot::Robot, std::shared_ptr<aikido::robot::Robot>>(m, "Robot")
      .def("getName",
           [](aikido::robot::Robot* self) -> std::string { return self->getName(); });
}

} // namespace python
} // namespace aikido
