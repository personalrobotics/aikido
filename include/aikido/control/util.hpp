#ifndef AIKIDO_CONTROL_COMMON_HPP_
#define AIKIDO_CONTROL_COMMON_HPP_

#include <chrono>

#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/MetaSkeleton.hpp>

namespace aikido {
namespace control {

/// Get joint names from skeleton for Executor intiialization
inline std::vector<std::string> skeletonToJointNames(
    const dart::dynamics::MetaSkeletonPtr skeleton)
{
  std::vector<std::string> ret;
  if (!skeleton)
    return ret;
  ret.reserve(skeleton->getNumDofs());
  for (const auto& dof : skeleton->getDofs())
  {
    ret.push_back(dof->getName());
  }
  return ret;
}

} // namespace control
} // namespace aikido

#endif // AIKIDO_CONTROL_COMMON_HPP_