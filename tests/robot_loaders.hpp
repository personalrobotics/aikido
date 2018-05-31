#ifndef AIKIDO_TESTS_ROBOT_LOADERS_HPP_
#define AIKIDO_TESTS_ROBOT_LOADERS_HPP_

#include <dart/dart.hpp>

namespace aikido {
namespace test {

const dart::common::Uri herbUrdfUri{
    "package://herb_description/robots/herb.urdf"};

dart::dynamics::SkeletonPtr loadRobot(const dart::common::Uri& robotUrdfUri,
                                      const dart::common::ResourceRetrieverPtr& retriever)
{
  dart::utils::DartLoader urdfLoader;
  dart::dynamics::SkeletonPtr robot = urdfLoader.parseSkeleton(robotUrdfUri,
                                                               retriever);  
  if (!robot)
  {
    throw std::runtime_error("Unable to load robot model.");
  }
  return robot;
}

} // namespace tests
} // namespace aikido

#endif // ifndef AIKIDO_TESTS_ROBOT_LOADERS_HPP_
