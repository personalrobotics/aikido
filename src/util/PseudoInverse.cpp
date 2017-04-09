#include <aikido/util/PseudoInverse.hpp>

#include <iostream>
#include <limits>
#include <memory>
#include <Eigen/Dense>

namespace aikido {
namespace util {

//==============================================================================
Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& mat, double eps)
{
  if (mat.rows() == mat.cols() && mat.determinant() > eps)
    return mat.inverse();

  else
  {
    if (mat.cols() == 1)
    {
      if (mat.isApproxToConstant(0))
      {
        return Eigen::VectorXd::Zero(mat.rows());
      }

      return mat.transpose() / (pow(mat.norm(), 2));
    }

    /// Use SVD decomposition.
    Eigen::JacobiSVD<Eigen::MatrixXd> jacSVD(
        mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = jacSVD.matrixU();
    Eigen::MatrixXd V = jacSVD.matrixV();
    Eigen::VectorXd S = jacSVD.singularValues();

    Eigen::MatrixXd S_inv(Eigen::MatrixXd::Zero(mat.cols(), mat.rows()));

    for (int i = 0; i < S.rows(); i++)
    {
      if (S(i) > eps)
      {
        S_inv(i, i) = 1.0 / S(i);
      }
      else
      {
        S_inv(i, i) = 0;
      }
    }

    return V * S_inv * U.transpose();
  }
}

} // namespace util
} // namespace aikido
