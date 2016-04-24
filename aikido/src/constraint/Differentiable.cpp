#include <aikido/constraint/Differentiable.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
void Differentiable::getValueAndJacobian(
  const statespace::StateSpace::State* _s,
  std::pair<Eigen::VectorXd, Eigen::MatrixXd>& _out) const
{
  getValue(_s, _out.first);
  getJacobian(_s, _out.second);
}

}
}
