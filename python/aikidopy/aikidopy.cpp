#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

void eigen_geometry(py::module& m);

void aikidopy_common(py::module& m);
void aikidopy_statespace(py::module& m);
void aikidopy_constraint(py::module& m);
void aikidopy_planner(py::module& m);
void aikidopy_robot(py::module& m);
#ifdef AIKIDO_HAS_RVIZ
void aikidopy_rviz(py::module& m);
#endif // AIKIDO_HAS_RVIZ

PYBIND11_MODULE(aikidopy, m)
{
  py::module::import("numpy");

  m.doc() = "aikidopy is a Python library for solving robotic motion planning "
      "and decision making problems.";

  aikidopy_common(m);
  aikidopy_statespace(m);
  aikidopy_constraint(m);
  aikidopy_planner(m);
  aikidopy_robot(m);
#ifdef AIKIDO_HAS_RVIZ
  aikidopy_rviz(m);
#endif // AIKIDO_HAS_RVIZ
}

} // namespace python
} // namespace aikido
