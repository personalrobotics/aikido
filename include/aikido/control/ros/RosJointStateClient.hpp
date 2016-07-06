#ifndef AIKIDO_CONTROL_ROS_ROSJOINTSTATECLIENT_HPP_
#define AIKIDO_CONTROL_ROS_ROSJOINTSTATECLIENT_HPP_
#include <string>
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
    const std::string& _topicName);

  void spin();

private:
  void jointStateCallback(const sensor_msgs::JointState& _jointState);

  dart::dynamics::SkeletonPtr mSkeleton;
  ::ros::CallbackQueue mCallbackQueue;
  ::ros::NodeHandle mNodeHandle;
  ::ros::Subscriber mSubscriber;
};

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_ROSJOINTSTATECLIENT_HPP_
