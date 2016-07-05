#ifndef ROS_TRAJECTORYEXECUTOR_HPP
#define ROS_TRAJECTORYEXECUTOR_HPP
#include <aikido/control/TrajectoryExecutor.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

#include <future>
#include <mutex>
#include <condition_variable>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

namespace aikido {
namespace control {
namespace ros {

class RosTrajectoryExecutor : public aikido::control::TrajectoryExecutor
{
public:
  RosTrajectoryExecutor(
    ::dart::dynamics::SkeletonPtr skeleton, 
    ::ros::NodeHandle node,
    const std::string& serverName,
    double timestep);

  virtual ~RosTrajectoryExecutor();

  std::future<void> execute(trajectory::TrajectoryPtr _traj) override;

  /// Simulates mTraj. To be executed on a separate thread.
  void spin();

private:
  using TrajectoryActionClient
    = actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction>;
  using GoalHandle = TrajectoryActionClient::GoalHandle;

  void transitionCallback(GoalHandle _handle);

  ::ros::NodeHandle mNode;
  ::ros::CallbackQueue mCallbackQueue;
  TrajectoryActionClient mClient;
  TrajectoryActionClient::GoalHandle mGoalHandle;

  ::dart::dynamics::SkeletonPtr mSkeleton;
  double mTimestep;

  bool mInProgress;
  std::promise<void> mPromise;
  trajectory::TrajectoryPtr mTrajectory;

  std::mutex mMutex;
};

} // namespace ros
} // namespace control
} // namespace aikido

#endif
