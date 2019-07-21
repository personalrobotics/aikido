#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace aikido {
namespace python {

void CollisionFree(py::module& sm);
void TSR(py::module& sm);

void aikidopy_constraint_dart(py::module& m)
{
  auto sm = m.def_submodule("dart");

  CollisionFree(sm);
  TSR(sm);
}

} // namespace python
} // namespace aikido
