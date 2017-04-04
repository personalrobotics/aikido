#ifndef AIKIDO_CONTROL_ROS_ROSTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_ROS_ROSTRAJECTORYEXECUTOR_HPP_
#include <chrono>
#include <future>
#include <mutex>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <dart/dart.hpp>
#include <aikido/control/TrajectoryExecutor.hpp>
#include <aikido/trajectory/Trajectory.hpp>

// actionlib and DART both #define this macro.
#undef DEPRECATED
#include <actionlib/client/action_client.h>
#undef DEPRECATED

namespace aikido {
namespace control {
namespace ros {

class RosTrajectoryExecutor : public aikido::control::TrajectoryExecutor
{
public:
  RosTrajectoryExecutor(
    ::dart::dynamics::MetaSkeletonPtr skeleton, 
    ::ros::NodeHandle node,
    const std::string& serverName,
    double timestep,
    double goalTimeTolerance,
    std::chrono::milliseconds connectionTimeout
      = std::chrono::milliseconds{1000},
    std::chrono::milliseconds connectionPollingRate
      = std::chrono::milliseconds{20}
  );

  virtual ~RosTrajectoryExecutor();

  std::future<void> execute(trajectory::TrajectoryPtr _traj) override;

  std::future<void> execute(
    trajectory::TrajectoryPtr _traj, const ::ros::Time& _startTime);

  /// Simulates mTraj. To be executed on a separate thread.
  void spin();

private:
  using TrajectoryActionClient
    = actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction>;
  using GoalHandle = TrajectoryActionClient::GoalHandle;

  bool waitForServer();

  void transitionCallback(GoalHandle _handle);

  ::ros::NodeHandle mNode;
  ::ros::CallbackQueue mCallbackQueue;
  TrajectoryActionClient mClient;
  TrajectoryActionClient::GoalHandle mGoalHandle;

  ::dart::dynamics::MetaSkeletonPtr mSkeleton;
  double mTimestep;
  double mGoalTimeTolerance;

  std::chrono::milliseconds mConnectionTimeout;
  std::chrono::milliseconds mConnectionPollingRate;

  bool mInProgress;
  std::promise<void> mPromise;
  trajectory::TrajectoryPtr mTrajectory;

  std::mutex mMutex;
};

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_ROSTRAJECTORYEXECUTOR_HPP_
