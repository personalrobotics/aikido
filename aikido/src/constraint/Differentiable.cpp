#include <aikido/constraint/Differentiable.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
std::pair<Eigen::VectorXd, Eigen::MatrixXd> Differentiable::getValueAndJacobian(
  const statespace::StateSpace::State* _s) const
{
  return std::make_pair<Eigen::VectorXd, Eigen::MatrixXd>(
    getValue(_s), getJacobian(_s));
}

}
}
