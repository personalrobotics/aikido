#ifndef AIKIDO_STATESPACE_JACOBIAN_H
#define AIKIDO_STATESPACE_JACOBIAN_H
#include <Eigen/Dense>

namespace aikido {
namespace statespace {

class Jacobian
{
};

class UtilJacobian
{
public:
  UtilJacobian(const Eigen::MatrixXd& _jac)
    : mJacobian(_jac)
  {
  }

protected:
  Eigen::MatrixXd mJacobian;
};

} // namespace statespace
} // namespace aikido

#endif
