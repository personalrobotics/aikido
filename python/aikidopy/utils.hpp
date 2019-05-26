#ifndef AIKIDOPY_UTILS_HPP_
#define AIKIDOPY_UTILS_HPP_

#include <vector>
#include <Eigen/Geometry>

namespace aikido {
namespace python {

Eigen::Isometry3d vectorToIsometry(const std::vector<double>& poseVector);

} // namespace python
} // namespace aikido

#endif // AIKIDOPY_UTILS_HPP_
