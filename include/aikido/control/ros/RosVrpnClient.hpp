#ifndef AIKIDO_CONTROL_ROS_ROSVRPNCLIENT_HPP_
#define AIKIDO_CONTROL_ROS_ROSVRPNCLIENT_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace aikido {
namespace control {
namespace ros {

/// Client that listens for PoseStamped messages for a list of rigid bodies and
/// provides a method for extracting the most recent position of each body.
class RosVrpnClient
{
public:
  /// Constructor.
  /// \param rigidBodies Rigid bodies to read Pose updates for.
  /// \param nodeHandle ROS node.
  /// \param capacity Number of PoseRecords that are saved per rigid body.
  RosVrpnClient(
      std::vector<std::string> rigidBodies,
      ::ros::NodeHandle nodeHandle,
      std::size_t capacity);

  /// Update mBuffer with any PoseStamped messages that have been received.
  void spin();

  /// Returns the most recent pose of each rigid body.
  ///
  /// \param rigidBodies Rigid bodies to read Pose updates for
  /// \return vector of poses for each rigid body
  std::vector<geometry_msgs::Pose> getLatestPosition(
      const std::vector<std::string>& rigidBodies) const;

private:
  struct PoseRecord
  {
    inline bool isValid() const
    {
      return mStamp.isValid();
    }

    ::ros::Time mStamp;
    geometry_msgs::Pose mPose;
  };

  /// Callback to add a new Pose to mBuffer
  ///
  /// \param rigidBody Rigid body to update
  /// \param msg New PoseStamped message to add to mBuffer
  void poseCallback(
      const std::string& rigidBody,
      const geometry_msgs::PoseStampedConstPtr& pose);

  mutable std::mutex mMutex;

  std::vector<std::string> mRigidBodies;
  std::unordered_map<std::string, boost::circular_buffer<PoseRecord>> mBuffer;
  std::size_t mCapacity;

  ::ros::CallbackQueue mCallbackQueue;
  ::ros::NodeHandle mNodeHandle;
  std::vector<::ros::Subscriber> mSubscribers;
};

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_ROSVRPNCLIENT_HPP_
