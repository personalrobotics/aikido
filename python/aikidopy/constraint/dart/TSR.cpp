#include <pybind11/pybind11.h>
#include <aikido/constraint.hpp>
#include <aikido/constraint/dart/TSR.hpp>
#include "utils.hpp"

namespace py = pybind11;

namespace aikido {
namespace python {

void TSR(py::module& m)
{
  py::class_<aikido::constraint::dart::TSR, std::shared_ptr<aikido::constraint::dart::TSR>>(m, "TSR")
  .def("setPose",
    [](aikido::constraint::dart::TSR* self,
          std::vector<double> pose) // x, y, z, qw, qx, qy, qz)
    {
      self->mT0_w = vectorToIsometry(pose);
    })
  .def("multiplyToEndEffectorTransform",
    [](aikido::constraint::dart::TSR* self,
        Eigen::Isometry3d endEffectorTransform)
    {
      self->mTw_e.matrix() *= endEffectorTransform.matrix();
    })
  .def("getPose",
    [](aikido::constraint::dart::TSR* self)
    -> Eigen::Isometry3d
    {
      return self->mT0_w;
    })
  .def("getEndEffectorTransform",
    [](aikido::constraint::dart::TSR* self)
    -> Eigen::Isometry3d
    {
      return self->mTw_e;
    });
}

} // namespace python
} // namespace aikido
