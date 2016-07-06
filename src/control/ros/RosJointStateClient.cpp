#include <aikido/control/ros/RosJointStateClient.hpp>

namespace aikido {
namespace control {
namespace ros {

//=============================================================================
RosJointStateClient::RosJointStateClient(
      dart::dynamics::SkeletonPtr _skeleton,
      ::ros::NodeHandle _nodeHandle,
      const std::string& _topicName)
  : mSkeleton{std::move(_skeleton)}
  , mCallbackQueue{} // Must be before mNodeHandle for order of destruction.
  , mNodeHandle{std::move(_nodeHandle)}
{
  mNodeHandle.setCallbackQueue(&mCallbackQueue);
  mSubscriber = mNodeHandle.subscribe(_topicName, 1,
    &RosJointStateClient::jointStateCallback, this);
}

//=============================================================================
void RosJointStateClient::spin()
{
  std::lock_guard<std::mutex> lock(mSkeleton->getMutex());

  mCallbackQueue.callAvailable();
}

//=============================================================================
void RosJointStateClient::jointStateCallback(
  const sensor_msgs::JointState& _jointState)
{
  // This method assumes that mSkeleton->getMutex() is locked.

  if (_jointState.position.size() != _jointState.name.size())
  {
    ROS_WARN_STREAM("Incorrect number of positions: expected "
      << _jointState.name.size() << ", got "
      << _jointState.position.size() << ".");
    return;
  }
  // TODO: Also check for velocities.

  for (size_t i = 0; i < _jointState.name.size(); ++i)
  {
    const auto dof = mSkeleton->getDof(_jointState.name[i]);
    if (dof)
    {
      dof->setPosition(_jointState.position[i]);
      // TODO: Also update velocities, if they are present.
    }

  }
}

} // namespace ros
} // namespace control
} // namespace aikido
