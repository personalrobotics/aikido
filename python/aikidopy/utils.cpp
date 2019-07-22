#include "utils.hpp"

namespace aikido {
namespace python {

Eigen::Isometry3d vectorToIsometry(const std::vector<double>& poseVector)
{
  Eigen::Map<const Eigen::VectorXd> p(poseVector.data(), static_cast<int>(poseVector.size()));
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = p.head<3>();
  pose.linear() = Eigen::Quaterniond(p[3], p[4], p[5], p[6]).toRotationMatrix();
  return pose;
}

} // namespace python
} // namespace aikido
