#include "aikido/robot/ros/RosRobot.hpp"

#include <srdfdom/model.h>
#include <urdf/model.h>

namespace aikido {
namespace robot {
namespace ros {

namespace internal {

inline dart::dynamics::MetaSkeletonPtr skeletonFromURDF(
    const dart::common::Uri& urdf,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  dart::utils::DartLoader urdfLoader;

  auto ret = urdfLoader.parseSkeleton(urdf, retriever);
  if (!ret)
  {
    throw std::runtime_error("Unable to load the robot from URDF.");
  }

  return ret;
}

} // namespace internal

//==============================================================================
RosRobot::RosRobot(
    const dart::common::Uri& urdf,
    const dart::common::Uri& srdf,
    const std::string name,
    const dart::common::ResourceRetrieverPtr& retriever)
  : Robot(internal::skeletonFromURDF(urdf, retriever), name)
{
  // Read the SRDF for disabled collision pairs.
  urdf::Model urdfModel;
  std::string urdfAsString = retriever->readAll(urdf);
  urdfModel.initString(urdfAsString);

  srdf::Model srdfModel;
  std::string srdfAsString = retriever->readAll(srdf);
  srdfModel.initString(urdfModel, srdfAsString);
  auto disabledCollisions = srdfModel.getDisabledCollisionPairs();
  for (auto disabledPair : disabledCollisions)
  {
    auto body0 = mMetaSkeleton->getBodyNode(disabledPair.link1_);
    auto body1 = mMetaSkeleton->getBodyNode(disabledPair.link2_);
    mSelfCollisionFilter->addBodyNodePairToBlackList(body0, body1);
  }
}

} // namespace ros
} // namespace robot
} // namespace aikido