#ifndef AIKIDO_CONTROL_ROS_ROSJOINTSTATECLIENT_HPP_
#define AIKIDO_CONTROL_ROS_ROSJOINTSTATECLIENT_HPP_
#include <string>
#include <boost/circular_buffer.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>

namespace aikido {
namespace control {
namespace ros {

class RosJointStateClient
{
public:
  RosJointStateClient(
    dart::dynamics::SkeletonPtr _skeleton,
    ::ros::NodeHandle _nodeHandle,
    const std::string& _topicName,
    size_t capacity);

  void spin();

  Eigen::VectorXd getLatestPosition(
    const dart::dynamics::MetaSkeleton& _metaSkeleton) const;

private:
  struct JointStateRecord
  {
    inline bool isValid() const { return mStamp.isValid(); }

    ::ros::Time mStamp;
    double mPosition;
  };

  void jointStateCallback(const sensor_msgs::JointState& _jointState);

  mutable std::mutex mMutex;

  dart::dynamics::SkeletonPtr mSkeleton;
  std::unordered_map<std::string,
    boost::circular_buffer<JointStateRecord>> mBuffer;
  size_t mCapacity;

  ::ros::CallbackQueue mCallbackQueue;
  ::ros::NodeHandle mNodeHandle;
  ::ros::Subscriber mSubscriber;
};

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_ROSJOINTSTATECLIENT_HPP_
