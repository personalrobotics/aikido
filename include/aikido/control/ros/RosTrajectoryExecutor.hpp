#ifndef AIKIDO_CONTROL_ROS_ROSTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_ROS_ROSTRAJECTORYEXECUTOR_HPP_
#include <chrono>
#include <future>
#include <mutex>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <aikido/control/TrajectoryExecutor.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <actionlib/client/action_client.h>

namespace aikido {
namespace control {
namespace ros {

/// This class sends trajectory commands to ROS server.
class RosTrajectoryExecutor : public aikido::control::TrajectoryExecutor
{
public:
  /// Constructor.
  /// \param[in] space Space in which trajectries operate.
  /// \param[in] node ROS node handle for action client.
  /// \param[in] serverName Name of the server to send traejctory to.
  /// \param[in] timestep Step size for interpolating trajectories.
  /// \param[in] goalTimeTolerance
  /// \param[in] connectionTimeout Timeout for server connection.
  /// \param[in] connectionPollingPeriod Polling period for server connection.
  template <typename DurationA, typename DurationB>
  RosTrajectoryExecutor(
    statespace::dart::MetaSkeletonStateSpacePtr space,
    ::ros::NodeHandle node,
    const std::string& serverName,
    double timestep,
    double goalTimeTolerance,
    const DurationA& connectionTimeout = std::chrono::milliseconds{1000},
    const DurationB& connectionPollingPeriod = std::chrono::milliseconds{20}
  );

  virtual ~RosTrajectoryExecutor();

  /// Sends trajectory to ROS server for execution.
  /// \param[in] traj Trajecotry to be executed.
  std::future<void> execute(trajectory::TrajectoryPtr traj) override;

  /// Sends trajectory to ROS server for execution.
  /// \param[in] traj Trajectrory to be executed.
  /// \param[in] startTime Start time for the trajectory.
  std::future<void> execute(
    trajectory::TrajectoryPtr traj, const ::ros::Time& startTime);

  /// To be executed on a separate thread.
  /// Regularly checks for the completion of a sent trajectory.
  void spin();

private:
  using TrajectoryActionClient
    = actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction>;
  using GoalHandle = TrajectoryActionClient::GoalHandle;

  bool waitForServer();

  void transitionCallback(GoalHandle handle);

  statespace::dart::MetaSkeletonStateSpacePtr mSpace;
  ::ros::NodeHandle mNode;
  ::ros::CallbackQueue mCallbackQueue;
  TrajectoryActionClient mClient;
  TrajectoryActionClient::GoalHandle mGoalHandle;

  double mTimestep;
  double mGoalTimeTolerance;

  std::chrono::milliseconds mConnectionTimeout;
  std::chrono::milliseconds mConnectionPollingPeriod;

  bool mInProgress;
  std::promise<void> mPromise;
  trajectory::TrajectoryPtr mTrajectory;

  std::mutex mMutex;
};

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_ROSTRAJECTORYEXECUTOR_HPP_

