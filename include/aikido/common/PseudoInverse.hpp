#ifndef AIKIDO_COMMON_PSEUDOINVERSE_HPP_
#define AIKIDO_COMMON_PSEUDOINVERSE_HPP_

#include <Eigen/Dense>

namespace aikido {
namespace common {

/// Computes the Moore-Penrose pseudoinverse of a matrix.
///
/// \param mat input matrix
/// \return pseudo-inverse of \c mat
Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& mat, double eps = 1e-6);

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_PSEUDOINVERSE_HPP_
