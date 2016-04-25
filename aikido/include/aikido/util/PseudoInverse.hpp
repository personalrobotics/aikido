#ifndef AIKIDO_UTIL_PSEUDOINVERSE_HPP_
#define AIKIDO_UTIL_PSEUDOINVERSE_HPP_

#include <memory>
#include <Eigen/Dense>

namespace aikido {
namespace util {

/// Computes the Moore-Penrose pseudoinverse of a matrix.
///
/// \param mat input matrix
/// \return pseudo-inverse of \c mat
Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& mat);

} // namespace util
} // namespace aikido

#endif // ifndef AIKIDO_UTIL_PSEUDOINVERSE_HPP_
