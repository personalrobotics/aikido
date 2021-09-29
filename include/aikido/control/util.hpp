#ifndef AIKIDO_CONTROL_UTIL_HPP_
#define AIKIDO_CONTROL_UTIL_HPP_

#include <set>

#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/MetaSkeleton.hpp>

namespace aikido {
namespace control {

/// Get joint names from skeleton for Executor intiialization
inline std::vector<std::string> skeletonToDofNames(
    dart::dynamics::ConstMetaSkeletonPtr skeleton)
{
  std::vector<std::string> names;
  if (!skeleton)
    return names;
  names.reserve(skeleton->getNumDofs());
  for (const auto& dof : skeleton->getDofs())
  {
    names.push_back(dof->getName());
  }
  return names;
}

/// Concatenate two sets of ExecutorTypes
/// Useful for initializer-list constructors
inline std::set<ExecutorType> concatenateTypes(
    std::set<ExecutorType> first, std::set<ExecutorType> second)
{
  std::set<ExecutorType> ret;
  ret.insert(first.begin(), first.end());
  ret.insert(second.begin(), second.end());
  return ret;
}

/// Check if MetaSkeletonPtr is null
/// Useful for initializer Lists
template <typename T>
inline T checkNull(T obj)
{
  if (!obj)
  {
    throw std::invalid_argument("Object is null.");
  }
  return obj;
}

} // namespace control
} // namespace aikido

#endif // AIKIDO_CONTROL_UTIL_HPP_