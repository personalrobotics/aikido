#ifndef AIKIDO_CONTROL_ROS_ROSJOINTSTATECLIENT_HPP_
#define AIKIDO_CONTROL_ROS_ROSJOINTSTATECLIENT_HPP_

#include <string>
#include <boost/circular_buffer.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace aikido {
namespace control {
namespace ros {

/// Client that listens for JointState messages for each skeleton joint and
/// provides a method for extracting the most recent position of each joint.
class RosJointStateClient
{
public:
  /// Constructor.
  /// \param _skeleton Skeleton to read JointState updates for.
  /// \param _nodeHandle ROS node.
  /// \param _topicName Name of topic to subscribe to for JointState updates.
  /// \param _capacity Number of JointStateRecords that are saved per joint.
  RosJointStateClient(
      dart::dynamics::SkeletonPtr _skeleton,
      ::ros::NodeHandle _nodeHandle,
      const std::string& _topicName,
      size_t capacity);

  /// Update mBuffer with any JointState messages that have been received.
  void spin();

  /// Returns the most recent position of each joint in _metaSkeleton.
  /// \param _metaSkeleton Skeleton to read DOFs from.
  /// \return vector of positions for each DOF
  Eigen::VectorXd getLatestPosition(
      const dart::dynamics::MetaSkeleton& _metaSkeleton) const;

  // TODO: implement
  // getPositionAtTime(const MetaSkeleton&, const ros::Time&, bool)
  // that interpolates position at the specified time, optionally blocking for
  // new data.
private:
  struct JointStateRecord
  {
    inline bool isValid() const
    {
      return mStamp.isValid();
    }

    ::ros::Time mStamp;
    double mPosition;
  };

  /// Callback to add a new JointState to mBuffer
  /// \param _jointState New JointState to add to mBuffer
  void jointStateCallback(const sensor_msgs::JointState& _jointState);

  mutable std::mutex mMutex;

  dart::dynamics::SkeletonPtr mSkeleton;
  std::unordered_map<std::string, boost::circular_buffer<JointStateRecord>>
      mBuffer;
  size_t mCapacity;

  ::ros::CallbackQueue mCallbackQueue;
  ::ros::NodeHandle mNodeHandle;
  ::ros::Subscriber mSubscriber;
};

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_ROSJOINTSTATECLIENT_HPP_
