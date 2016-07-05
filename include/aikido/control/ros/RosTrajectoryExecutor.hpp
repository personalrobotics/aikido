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
    std::chrono::milliseconds period,
    const std::string& serverName,
    ::ros::NodeHandle node,
    double trajTimeStep);

  virtual ~RosTrajectoryExecutor();

  std::future<void> execute(trajectory::TrajectoryPtr _traj) override;

private:
  using TrajectoryClient
    = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

  /// Simulates mTraj. To be executed on a separate thread.
  void spin();

  ::ros::NodeHandle mNode;

  ::dart::dynamics::SkeletonPtr mSkeleton;
  std::unique_ptr<std::promise<void>> mPromise;
  trajectory::TrajectoryPtr mTraj;

  std::string mServerName;

  /// The time resolution at which to publish trajectory
  double mTrajTimeStep;

  /// spin()'s trajectory execution cycle.
  std::chrono::milliseconds mPeriod;

  /// Blocks spin() until execute(...) is called; paired with mSpinLock.
  std::condition_variable mCv;

  /// Lock for keeping spin thread alive and executing a trajectory. 
  /// Manages access on mTraj, mPromise, mRunning
  std::mutex mSpinMutex;

  /// Thread for spin().
  std::thread mThread;

  /// Flag for killing spin thread. 
  bool mRunning;

  TrajectoryClient mTrajClientPtr;
};

} // namespace ros
} // namespace control
} // namespace aikido

#endif
