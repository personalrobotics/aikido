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
    const bool addDefaultExecutors,
    const dart::common::ResourceRetrieverPtr& retriever,
    std::shared_ptr<::ros::NodeHandle> node)
  : Robot(
      aikido::io::loadSkeletonFromURDF(retriever, urdf),
      name,
      addDefaultExecutors)
  , mRosLoadControllerServiceClient(nullptr)
  , mRosSwitchControllerServiceClient(nullptr)
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

//==============================================================================
void RosRobot::step(const std::chrono::system_clock::time_point& timepoint)
{
  // Lock only if root robot
  std::unique_ptr<std::lock_guard<std::mutex>> lock;
  if (!mParentRobot)
  {
    lock = std::make_unique<std::lock_guard<std::mutex>>(
        getRootSkeleton()->getMutex());
  }

  if(mRosJointModeCommandClient)
    mRosJointModeCommandClient->step();

  if (!mActiveExecutor.empty())
  {
    mExecutors[mActiveExecutor]->step(timepoint);
  }

  for (const auto& subrobot : mSubRobots)
  {
    subrobot.second->step(timepoint);
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
      executor, std::string(), hardware_interface::JointCommandModes::ERROR, desiredName);
}

//=============================================================================
bool RosRobot::activateExecutor(std::string id)
{
  // Deactivate active executor
  deactivateExecutor();

  // Validate input
  if (mExecutors.find(id) == mExecutors.end())
  {
    return false;
  }

  if (!mRosControllerNames[id].empty()
      && !startController(mRosControllerNames[id]))
  {
    throw std::runtime_error(
        "Could not start controller: " + mRosControllerNames[id]);
  }

  if (mRosControllerModes[id]
          != hardware_interface::JointCommandModes::ERROR
      && !switchControllerMode(mRosControllerModes[id]))
  {
    throw std::runtime_error(
        "Could not switch controller mode to: "
        + aikido::control::ros::intFromMode(
            mRosControllerModes[id]));
  }

  return Robot::activateExecutor(id);
}

//=============================================================================
void RosRobot::setRosLoadControllerServiceClient(
      const std::shared_ptr<::ros::ServiceClient>& rosLoadControllerServiceClient)
{
  mRosLoadControllerServiceClient = rosLoadControllerServiceClient;
}

//=============================================================================
void RosRobot::setRosSwitchControllerServiceClient(
      const std::shared_ptr<::ros::ServiceClient>& rosSwitchControllerServiceClient)
{
  mRosSwitchControllerServiceClient = rosSwitchControllerServiceClient;
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
  if (!mRosLoadControllerServiceClient)
  {
    throw std::runtime_error(
        "ROS controller manager service client not instantiated.");
  }

  controller_manager_msgs::LoadController srv;
  srv.request.name = loadControllerName;

  return mRosLoadControllerServiceClient->call(srv) && srv.response.ok;
}

//=============================================================================
bool RosRobot::startController(const std::string startControllerName)
{
  return switchControllers(std::vector<std::string>{startControllerName}, std::vector<std::string>{});
}

//=============================================================================
bool RosRobot::stopController(const std::string stopControllerName)
{
  cancelAllCommands();
  return switchControllers(std::vector<std::string>{}, std::vector<std::string>{stopControllerName});
}

//=============================================================================
bool RosRobot::switchControllers(
    const std::vector<std::string> startControllersNames, const std::vector<std::string> stopControllersNames)
{
  if (!mRosSwitchControllerServiceClient)
  {
    throw std::runtime_error(
        "ROS controller manager service client not instantiated.");
  }

  controller_manager_msgs::SwitchController srv;
  // First try stopping the started controllers
  // Avoids us falsely detecting a failure if already started
  srv.request.stop_controllers = startControllersNames;
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  mRosSwitchControllerServiceClient->call(srv); // Don't care about response code

  // Actual command
  srv.request.start_controllers = startControllersNames;  
  srv.request.stop_controllers = stopControllersNames;
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::STRICT;

  return mRosSwitchControllerServiceClient->call(srv) && srv.response.ok;
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