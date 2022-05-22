#include "aikido/robot/ros/RosRobot.hpp"

#include <srdfdom/model.h>
#include <urdf/model.h>

#include "aikido/io/util.hpp"

namespace aikido {
namespace robot {
namespace ros {

//==============================================================================
RosRobot::RosRobot(
    const dart::common::Uri& urdf,
    const dart::common::Uri& srdf,
    const std::string name,
    const dart::common::ResourceRetrieverPtr& retriever,
    std::shared_ptr<::ros::NodeHandle> node)
  : Robot(aikido::io::loadSkeletonFromURDF(retriever, urdf), name, false)
  , mRosControllerServiceClient(nullptr)
  , mRosJointModeCommandClient(nullptr)
{
  // Set up ROS Node
  if (!node)
  {
    mNode = std::make_shared<::ros::NodeHandle>();
  }
  else
  {
    mNode = node;
  }

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

//=============================================================================
RobotPtr RosRobot::registerSubRobot(
    dart::dynamics::ReferentialSkeletonPtr refSkeleton, const std::string& name)
{
  // Ensure name is unique
  if (mSubRobots.find(name) != mSubRobots.end())
  {
    dtwarn << "Subrobot '" << name << "' already exists." << std::endl;
    return nullptr;
  }

  // Ensure all body nodes in skeleton are owned by this robot
  for (auto bodyNode : refSkeleton->getBodyNodes())
  {
    if (!mMetaSkeleton->hasBodyNode(bodyNode))
    {
      dtwarn << "Subrobot '" << name << "'' contains body node "
             << bodyNode->getName() << " not in parent MetaSkeleton."
             << std::endl;
      return nullptr;
    }
  }

  // Ensure subrobot DoFs are disjoint
  for (const auto& subrobot : mSubRobots)
  {
    auto dofs = subrobot.second->getDofs();
    for (std::string dofName : util::dofNamesFromSkeleton(refSkeleton))
    {
      if (dofs.find(dofName) != dofs.end())
      {
        dtwarn << "Subrobot '" << name << "'' overlaps existing subrobot "
               << subrobot.first << " at DoF " << dofName << "." << std::endl;
        return nullptr;
      }
    }
  }

  // Create the sub RosRobot.
  auto subRosRobot = std::make_shared<RosRobot>(
      refSkeleton, this, mCollisionDetector, mSelfCollisionFilter, name);
  mSubRobots[name] = subRosRobot;
  return subRosRobot;
}

//=============================================================================
void RosRobot::deactivateExecutor()
{
  if (!mActiveExecutor.empty())
  {
    if (!mRosControllerNames[mActiveExecutor].empty()
        && !stopController(mRosControllerNames[mActiveExecutor]))
    {
      throw std::runtime_error(
          "Could not stop controller: " + mRosControllerNames[mActiveExecutor]);
    }
    Robot::deactivateExecutor();
  }
}

//=============================================================================
std::string RosRobot::registerExecutor(
    aikido::control::ExecutorPtr executor,
    std::string controllerName,
    hardware_interface::JointCommandModes controllerMode,
    std::string desiredName)
{
  if (!controllerName.empty() && !loadController(controllerName))
  {
    throw std::runtime_error("Could not load controller: " + controllerName);
  }
  std::string id = Robot::registerExecutor(executor,desiredName);
  if (!id.empty())
  {
    mRosControllerNames[id] = controllerName;
    mRosControllerModes[id] = controllerMode;
  }
  return id;
}

std::string RosRobot::registerExecutor(
    aikido::control::ExecutorPtr executor, 
    std::string controllerName,
    std::string desiredName)
{
  return registerExecutor(
      executor, controllerName, hardware_interface::JointCommandModes::ERROR, desiredName);
}

//=============================================================================
std::string RosRobot::registerExecutor(aikido::control::ExecutorPtr executor, std::string desiredName)
{
  return registerExecutor(
      executor, std::string(), hardware_interface::JointCommandModes::ERROR,desiredName);
}

//=============================================================================
bool RosRobot::activateExecutor(std::string id)
{
  // Deactivate active executor
  deactivateExecutor();
  if (!mRosControllerNames[mActiveExecutor].empty()
      && !startController(mRosControllerNames[mActiveExecutor]))
  {
    throw std::runtime_error(
        "Could not start controller: " + mRosControllerNames[mActiveExecutor]);
  }

  if (mRosControllerModes[mActiveExecutor]
          != hardware_interface::JointCommandModes::ERROR
      && !switchControllerMode(mRosControllerModes[mActiveExecutor]))
  {
    throw std::runtime_error(
        "Could not switch controller mode to: "
        + aikido::control::ros::intFromMode(
            mRosControllerModes[mActiveExecutor]));
  }

  return Robot::activateExecutor(id);
}

//=============================================================================
void RosRobot::setRosControllerServiceClient(
      const std::shared_ptr<::ros::ServiceClient>& rosControllerServiceClient)
{
  mRosControllerServiceClient = rosControllerServiceClient;
}

//=============================================================================
void RosRobot::setRosJointModeCommandClient(
      const std::shared_ptr<aikido::control::ros::RosJointModeCommandClient>& rosJointModeCommandClient)
{
  mRosJointModeCommandClient = rosJointModeCommandClient;
}

//=============================================================================
bool RosRobot::loadController(const std::string loadControllerName)
{
  if (!mRosControllerServiceClient)
  {
    throw std::runtime_error(
        "ROS controller manager service client not instantiated.");
  }

  controller_manager_msgs::LoadController srv;
  srv.request.name = loadControllerName;

  return mRosControllerServiceClient->call(srv) && srv.response.ok;
}

//=============================================================================
bool RosRobot::startController(const std::string startControllerName)
{
  return switchController(startControllerName, std::string());
}

//=============================================================================
bool RosRobot::stopController(const std::string stopControllerName)
{
  cancelAllCommands();
  return switchController(std::string(), stopControllerName);
}

//=============================================================================
bool RosRobot::switchController(
    const std::string startControllerName, const std::string stopControllerName)
{
  if (!mRosControllerServiceClient)
  {
    throw std::runtime_error(
        "ROS controller manager service client not instantiated.");
  }

  controller_manager_msgs::SwitchController srv;
  // First try stopping the started controllers
  // Avoids us falsely detecting a failure if already started
  srv.request.stop_controllers = std::vector<std::string>{startControllerName};
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  mRosControllerServiceClient->call(srv); // Don't care about response code

  // Actual command
  srv.request.start_controllers = std::vector<std::string>{startControllerName};
  srv.request.stop_controllers = std::vector<std::string>{stopControllerName};
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::STRICT;

  return mRosControllerServiceClient->call(srv) && srv.response.ok;
}

//=============================================================================
bool RosRobot::switchControllerMode(
    const hardware_interface::JointCommandModes jointMode)
{
  if (!mRosJointModeCommandClient)
  {
    throw std::runtime_error(
        "ROS joint mode controller actionlib client not instantiated.");
  }

  // Currently we only send the same control mode to each joint
  auto future = mRosJointModeCommandClient->execute(
      std::vector<hardware_interface::JointCommandModes>(
          1, jointMode));
  try
  {
    future.get();
  }
  catch (const std::exception& e)
  {
    dtwarn << "Exception in controller mode switching: " << e.what()
           << std::endl;
    return false;
  }
  return true;
}

} // namespace ros
} // namespace robot
} // namespace aikido