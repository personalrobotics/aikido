#ifndef AIKIDO_UTIL_PSEUDOINVERSE_H_
#define AIKIDO_UTIL_PSEUDOINVERSE_H_

#include <memory>
#include <Eigen/Dense>

namespace aikido {
namespace util {

Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd& mat);


}
}

#endif

