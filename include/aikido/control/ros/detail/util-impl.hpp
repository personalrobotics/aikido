#include <chrono>
#include <mutex>
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
  PeriodDuration periodDuration = std::chrono::milliseconds{ 10 }
)
{
  using Clock = std::chrono::steady_clock;

  const auto startTime = Clock::now();
  const auto endTime = startTime + timeoutDuration;
  auto currentTime = startTime + periodDuration;

  while(currentTime < endTime)
  {
    callbackQueue.callAvailable();

    // TODO : Is this thread safe?
    if (actionClient.isServerConnected())
      return true;

    currentTime += periodDuration;
    std::this_thread::sleep_until(currentTime);
  }
}

} // namespace ros
} // namespace control
} // namespace aikido