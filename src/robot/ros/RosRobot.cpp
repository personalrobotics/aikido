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
    const std::string rosControllerManagerServerName,
    const std::string rosJointModeServerName)
  : Robot(aikido::io::loadSkeletonFromURDF(retriever, urdf), name)
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

  if(rosControllerManagerServerName != "")
  {
    mRosControllerServiceClient = std::make_unique<::ros::ServiceClient>(
        mNode->serviceClient<controller_manager_msgs::SwitchController>(rosControllerManagerServerName));
  }

  if(rosJointModeServerName != "")
  {
    std::vector<std::string> jointNames;
    for (auto joint : mMetaSkeleton->getDofs())
    {
      jointNames.push_back(joint->getName());
    }

    mRosJointModeCommandClient = std::make_unique<aikido::control::ros::RosJointModeCommandClient>(
        *mNode,rosJointModeServerName,
        jointNames);
  }
}

void RosRobot::deactivateExecutor()
{
  if (mActiveExecutor >= 0)
  {
    stopController(mRosControllerNames[mActiveExecutor]); // Rajat TODO: What if stop controller fails?
    Robot::deactivateExecutor();
  }
}

int RosRobot::registerExecutor(
    aikido::control::ExecutorPtr executor, 
    std::string controller_name, 
    hardware_interface::JointCommandModes controller_mode)
{
  int id = Robot::registerExecutor(executor);
  if(id != -1 && loadController(controller_name))
  {
    mRosControllerNames.push_back(controller_name);
    mRosControllerModes.push_back(controller_mode);
  }
  return id;
}

int RosRobot::registerExecutor(aikido::control::ExecutorPtr executor)
{
  return registerExecutor(executor,"",hardware_interface::JointCommandModes::ERROR);
}

bool RosRobot::activateExecutor(int id)
{
  // Deactivate active executor
  deactivateExecutor();
  if(Robot::activateExecutor(id))
  {
    return startController(mRosControllerNames[mActiveExecutor]) && switchControllerMode(mRosControllerModes[mActiveExecutor]);
  }
  return false;
}

bool RosRobot::activateExecutor(const aikido::control::ExecutorType type)
{
  for (int i = mExecutors.size() - 1; i >= 0; i--)
  {
    auto types = mExecutors[i]->getTypes();
    if (types.find(type) != types.end())
    {
      return activateExecutor(i);
    }
  }

  deactivateExecutor();
  return false;  
}

bool RosRobot::startController(const std::string startControllerName)
{
  return switchController(startControllerName, std::string());
}

bool RosRobot::stopController(const std::string stopControllerName)
{
  cancelAllCommands();
  return switchController(std::string(), stopControllerName);
}

bool RosRobot::switchController(const std::string startControllerName, const std::string stopControllerName)
{
  if (!mRosControllerServiceClient)
    throw std::runtime_error("ServiceClient not instantiated.");

  controller_manager_msgs::SwitchController srv;
  // First try stopping the started controllers
  // Avoids us falsely detecting a failure if already started
  srv.request.stop_controllers = std::vector<std::string>{startControllerName};
  srv.request.strictness = controller_manager_msgs::SwitchControllerRequest::BEST_EFFORT;
  mRosControllerServiceClient->call(srv); // Don't care about response code

  // Actual command
  srv.request.start_controllers = std::vector<std::string>{startControllerName};
  srv.request.stop_controllers = std::vector<std::string>{stopControllerName};
  srv.request.strictness = controller_manager_msgs::SwitchControllerRequest::STRICT;

  return mRosControllerServiceClient->call(srv) && srv.response.ok;
}

bool RosRobot::loadController(const std::string loadControllerName)
{
  if(loadControllerName == "")
    return true; // no controller to load

  controller_manager_msgs::LoadController srv;
  srv.request.name = loadControllerName; // Rajat ToDo: handle cases when this can fail

  return mRosControllerServiceClient->call(srv) && srv.response.ok;
}

bool RosRobot::switchControllerMode(const hardware_interface::JointCommandModes joint_mode)
{
  if(joint_mode == hardware_interface::JointCommandModes::ERROR)
    return false;
  
  auto future = mRosJointModeCommandClient->execute(std::vector<hardware_interface::JointCommandModes>(mMetaSkeleton->getDofs().size(),joint_mode));
  try 
  {
    future.get();
  } catch (const std::exception &e) {
    dtwarn << "Exception in controller mode switching: " << e.what() << std::endl;
    return false;
  }
  return true;
}

} // namespace ros
} // namespace robot
} // namespace aikido