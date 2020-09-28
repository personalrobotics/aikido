#include "aikido/robot/GenericRobot.hpp"

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <aikido/io/CatkinResourceRetriever.hpp>

namespace aikido {
namespace robot {

using constraint::dart::CollisionFreePtr;
using statespace::dart::ConstMetaSkeletonStateSpacePtr;

//==============================================================================
GenericRobot::GenericRobot(
    const dart::common::Uri& urdf,
    const dart::common::Uri& srdf,
    const std::string& name)
  : mName(name)
{
  // Read the URDF.
  const dart::common::ResourceRetrieverPtr& retriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();
  dart::utils::DartLoader urdfLoader;
  mMetaSkeleton = urdfLoader.parseSkeleton(urdf, retriever);
  if (!mMetaSkeleton)
  {
    throw std::runtime_error("Unable to load the robot from URDF.");
  }

  // Instantiate collision objects to disable allowed self-collisions.
  auto collisionDetector = dart::collision::FCLCollisionDetector::create();
  auto collideWith = collisionDetector->createCollisionGroupAsSharedPtr();
  auto selfCollisionFilter
      = std::make_shared<dart::collision::BodyNodeCollisionFilter>();

  // TODO: Avoid loading the URDF this second time. See PR63 [libherb].
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
    selfCollisionFilter->addBodyNodePairToBlackList(body0, body1);
  }
  mStateSpace = std::make_shared<statespace::dart::MetaSkeletonStateSpace>(
      mMetaSkeleton.get());
}

//==============================================================================
GenericRobot::GenericRobot(
    const dart::dynamics::MetaSkeletonPtr& skeleton, const std::string& name)
  : mName(name)
{
  mMetaSkeleton = skeleton;
}

//==============================================================================
std::future<void> GenericRobot::executeTrajectory(
    const trajectory::TrajectoryPtr& trajectory) const
{
  return mTrajectoryExecutor->execute(trajectory);
}

//==============================================================================
void GenericRobot::step(const std::chrono::system_clock::time_point& timepoint)
{
  // TODO(avk): Is the following statement valid? Where is it conveyed to the
  // user and what are its implications?:
  // Assumes that the parent robot is locked
  mTrajectoryExecutor->step(timepoint);
}

// ==============================================================================
CollisionFreePtr GenericRobot::getSelfCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const
{
  return nullptr;
}

//=============================================================================
constraint::TestablePtr GenericRobot::getFullCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const CollisionFreePtr& collisionFree) const
{
  return nullptr;
}

//=============================================================================
std::shared_ptr<GenericRobot> GenericRobot::registerChildRobot(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const std::string& name,
    const RobotType type)
{
  if (mChildren.find(name) != mChildren.end())
  {
    throw std::invalid_argument("Child is already present");
  }

  // Dispatch via factory.
  std::shared_ptr<GenericRobot> child;
  if (type == RobotType::Manipulator)
  {
    child = std::make_shared<GenericManipulator>(metaSkeleton, name);
  }
  else if (type == RobotType::WheeledBase)
  {
    // TODO(avk): Defaulting to Manipulator again.
    child = std::make_shared<GenericManipulator>(metaSkeleton, name);
  }
  // Update the root robot.
  const auto& root = mRootRobot ? mRootRobot : this;
  child->setRootRobot(root);
  mChildren[name] = child;
  return child;
}

//=============================================================================
std::shared_ptr<GenericRobot> GenericManipulator::registerChildRobot(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const std::string& name,
    const RobotType type)
{
  auto child = GenericRobot::registerChildRobot(metaSkeleton, name, type);
  if (type == RobotType::Gripper)
  {
    mGripper = std::dynamic_pointer_cast<GenericGripper>(child);
  }
}

} // namespace robot
} // namespace aikido
