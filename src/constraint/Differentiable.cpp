#include <aikido/constraint/Differentiable.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
void Differentiable::getValueAndJacobian(
    const statespace::StateSpace::State* _s,
    Eigen::VectorXd& _val,
    Eigen::MatrixXd& _jac) const
{
  getValue(_s, _val);
  getJacobian(_s, _jac);
}

} // namespace constraint
} // namespace aikido
