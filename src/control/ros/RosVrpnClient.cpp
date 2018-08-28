#include <aikido/control/ros/RosVrpnClient.hpp>

namespace aikido {
namespace control {
namespace ros {

//==============================================================================
RosVrpnClient::RosVrpnClient(
    std::vector<std::string> rigidBodies,
    ::ros::NodeHandle nodeHandle,
    std::size_t capacity)
  : mRigidBodies{rigidBodies}
  , mBuffer{}
  , mCapacity{capacity}
  , mCallbackQueue{} // Must be after mNodeHandle for order of destruction.
  , mNodeHandle{std::move(nodeHandle)}
{
  if (rigidBodies.size() == 0)
    throw std::invalid_argument("Rigid bodies must be non-empty.");

  if (capacity < 1)
    throw std::invalid_argument("Capacity must be positive.");

  mNodeHandle.setCallbackQueue(&mCallbackQueue);

  mSubscribers.reserve(mRigidBodies.size());
  for (const std::string& bodyName : mRigidBodies)
  {
    mSubscribers.push_back(
        mNodeHandle.subscribe<geometry_msgs::PoseStamped>(
            "/vrpn_client_node/" + bodyName + "/pose",
            1,
            std::bind(
                &RosVrpnClient::poseCallback,
                this,
                bodyName,
                std::placeholders::_1)));
  }
}

//==============================================================================
void RosVrpnClient::spin()
{
  std::lock_guard<std::mutex> bufferLock{mMutex};

  mCallbackQueue.callAvailable();
}

//==============================================================================
std::vector<geometry_msgs::Pose> RosVrpnClient::getLatestPosition(
    const std::vector<std::string>& rigidBodies) const
{
  std::lock_guard<std::mutex> bufferLock{mMutex};

  std::vector<geometry_msgs::Pose> poses;
  poses.reserve(rigidBodies.size());

  for (std::size_t i = 0; i < rigidBodies.size(); ++i)
  {
    const auto bodyName = rigidBodies[i];

    const auto it = mBuffer.find(bodyName);
    if (it == std::end(mBuffer))
    {
      std::stringstream msg;
      msg << "No data is available for '" << bodyName << "'.";
      throw std::runtime_error(msg.str());
    }

    const auto& buffer = it->second;
    if (buffer.empty())
    {
      std::stringstream msg;
      msg << "Data for '" << bodyName << "' is invalid.";
      throw std::runtime_error(msg.str());
    }

    const auto& record = buffer.back();
    poses[i] = record.mPose;
  }

  return poses;
}

//==============================================================================
void RosVrpnClient::poseCallback(
    const std::string& bodyName, const geometry_msgs::PoseStampedConstPtr& msg)
{
  // This method assumes that mMutex is locked.

  const auto result = mBuffer.emplace(
      bodyName, boost::circular_buffer<PoseRecord>{mCapacity});
  auto& buffer = result.first->second;

  if (!buffer.empty() && msg->header.stamp < buffer.back().mStamp)
  {
    // Ignore out of order Pose message.
    ROS_WARN_STREAM(
        "Ignoring out of order message: received timestamp of "
        << msg->header.stamp
        << " is before previously received timestamp of "
        << buffer.back().mStamp);

    return;
  }

  PoseRecord record;
  record.mStamp = msg->header.stamp;
  record.mPose = msg->pose;

  buffer.push_back(record);
}

} // namespace ros
} // namespace control
} // namespace aikido
