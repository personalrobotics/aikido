#include <aikido/control/ros/RosJointStateClient.hpp>

namespace aikido {
namespace control {
namespace ros {

//=============================================================================
RosJointStateClient::RosJointStateClient(
      dart::dynamics::SkeletonPtr _skeleton,
      ::ros::NodeHandle _nodeHandle,
      const std::string& _topicName,
      size_t capacity)
  : mSkeleton{std::move(_skeleton)}
  , mBuffer{}
  , mCapacity{capacity}
  , mCallbackQueue{} // Must be before mNodeHandle for order of destruction.
  , mNodeHandle{std::move(_nodeHandle)}
{
  if (!mSkeleton)
    throw std::invalid_argument("Skeleton is null.");

  if (capacity < 1)
    throw std::invalid_argument("Capacity must be positive.");

  mNodeHandle.setCallbackQueue(&mCallbackQueue);
  mSubscriber = mNodeHandle.subscribe(_topicName, 1,
    &RosJointStateClient::jointStateCallback, this);
}

//=============================================================================
void RosJointStateClient::spin()
{
  std::lock_guard<std::mutex> skeletonLock{mSkeleton->getMutex()};
  std::lock_guard<std::mutex> bufferLock{mMutex};

  mCallbackQueue.callAvailable();
}

//=============================================================================
Eigen::VectorXd RosJointStateClient::getLatestPosition(
  const dart::dynamics::MetaSkeleton& _metaSkeleton) const
{
  std::lock_guard<std::mutex> bufferLock{mMutex};
  Eigen::VectorXd position(_metaSkeleton.getNumDofs());

  for (size_t idof = 0; idof < _metaSkeleton.getNumDofs(); ++idof)
  {
    const auto dof = _metaSkeleton.getDof(idof);
    const auto it = mBuffer.find(dof->getName());
    if (it == std::end(mBuffer))
    {
      std::stringstream msg;
      msg << "No data is available for '" << dof->getName() << "'.";
      throw std::runtime_error(msg.str());
    }

    const auto& buffer = it->second;
    if (buffer.empty())
    {
      std::stringstream msg;
      msg << "Data for '" << dof->getName() << "' is invalid.";
      throw std::runtime_error(msg.str());
    }

    const auto& record = buffer.back();
    position[idof] = record.mPosition;
  }

  return position;
}

//=============================================================================
void RosJointStateClient::jointStateCallback(
  const sensor_msgs::JointState& _jointState)
{
  // This method assumes that mSkeleton->getMutex() and mMutex are locked.

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
    const auto result = mBuffer.emplace(_jointState.name[i],
      boost::circular_buffer<JointStateRecord>{mCapacity});
    auto& buffer = result.first->second;

    JointStateRecord record;
    record.mStamp = _jointState.header.stamp;
    record.mPosition = _jointState.position[i];

    // TODO: check that the data in buffer is sequential, i.e. that we did not
    // receive JointState messages out of order. It's probably safe to ignore
    // the message (and print a warning!) if this occurs.
    buffer.push_back(record);
  }
}

} // namespace ros
} // namespace control
} // namespace aikido
