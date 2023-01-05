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

  if (mActiveExecutor >= 0)
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
  if (!validateSubRobot(refSkeleton, name))
  {
    return nullptr;
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
  if (mActiveExecutor >= 0)
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
int RosRobot::registerExecutor(
    aikido::control::ExecutorPtr executor,
    std::string desiredName,
    std::string controllerName,
    hardware_interface::JointCommandModes controllerMode)
{
  int id = registerExecutor(executor, desiredName, controllerName);
  if (id >= 0 && controllerMode != hardware_interface::JointCommandModes::ERROR)
  {
    mRosControllerModes[id] = controllerMode;
  }
  return id;
}

int RosRobot::registerExecutor(
    aikido::control::ExecutorPtr executor,
    std::string desiredName, 
    std::string controllerName)
{
  if (!controllerName.empty() && !loadController(controllerName))
  {
    dtwarn << "Could not load controller: " + controllerName
           << std::endl;
    return -1;
  }
  int id = Robot::registerExecutor(executor, desiredName);
  if (id >= 0 && !controllerName.empty())
  {
    mRosControllerNames[id] = controllerName;
  }
  return id;
}

//=============================================================================
bool RosRobot::activateExecutor(const int id)
{
  // Deactivate active executor
  deactivateExecutor();

  // Validate input
  if (id < 0 || (size_t)id >= mExecutors.size())
  {
    dtwarn << "Could not activate executor as id is invalid."
           << std::endl;
    return false;
  }

  if (mRosControllerNames.find(id) != mRosControllerNames.end()
      && !startController(mRosControllerNames[id]))
  {
    dtwarn << "Could not start controller: " + mRosControllerNames[id]
           << std::endl;
    return false;
  }

  if (mRosControllerModes.find(id) != mRosControllerModes.end()
      && !switchControllerMode(mRosControllerModes[id]))
  {
    dtwarn << "Could not switch controller mode to: "
           + aikido::control::ros::intFromMode(mRosControllerModes[id])
           << std::endl;
    return false;
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
    dtwarn << "ROS controller manager service client not instantiated."
           << std::endl;
    return false;
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
    dtwarn << "ROS controller manager service client not instantiated."
           << std::endl;
    return false;
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
    dtwarn << "ROS joint mode controller actionlib client not instantiated."
           << std::endl;
    return false;
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
