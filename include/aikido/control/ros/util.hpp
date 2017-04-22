#ifndef AIKIDO_CONTROL_ROS_UTIL_HPP_
#define AIKIDO_CONTROL_ROS_UTIL_HPP_
#include <chrono>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/client/action_client.h>

namespace aikido {
namespace control {
namespace ros {

template <class ActionSpec, class TimeoutDuration, class PeriodDuration>
bool waitForActionServer(
  actionlib::ActionClient<ActionSpec>& actionClient,
  ::ros::CallbackQueue& callbackQueue,
  TimeoutDuration timeoutDuration = std::chrono::milliseconds{ 1000 },
  PeriodDuration periodDuration = std::chrono::milliseconds{ 10 } );


} // namespace ros
} // namespace control
} // namespace aikido

#include "detail/util-impl.hpp"

#endif // AIKIDO_CONTROL_ROS_UTIL_HPP_