#include <pybind11/pybind11.h>
#include <aikido/robot.hpp>

namespace py = pybind11;

namespace aikido {
namespace python {

void ConcreteManipulator(py::module& m)
{
  py::class_<aikido::robot::ConcreteManipulator, std::shared_ptr<aikido::robot::ConcreteManipulator>>(m, "ConcreteManipulator")
      .def("getName",
           [](aikido::robot::ConcreteManipulator* self) -> std::string { return self->getName(); });
}

} // namespace python
} // namespace aikido
