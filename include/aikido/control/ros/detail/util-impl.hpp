#include <chrono>
#include <mutex>

#include <actionlib/client/action_client.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace aikido {
namespace control {
namespace ros {

template <class ActionSpec, class TimeoutDuration, class PeriodDuration>
bool waitForActionServer(
    actionlib::ActionClient<ActionSpec>& actionClient,
    ::ros::CallbackQueue& callbackQueue,
    TimeoutDuration timeoutDuration,
    PeriodDuration periodDuration)
{
  using Clock = std::chrono::steady_clock;

  const auto startTime = Clock::now();
  const auto endTime = startTime + timeoutDuration;
  auto currentTime = startTime;

  while (currentTime < endTime)
  {
    callbackQueue.callAvailable();

    // TODO : Is this thread safe?
    if (actionClient.isServerConnected())
      return true;

    currentTime += periodDuration;
    std::this_thread::sleep_until(currentTime);
  }

  return false;
}

} // namespace ros
} // namespace control
} // namespace aikido
