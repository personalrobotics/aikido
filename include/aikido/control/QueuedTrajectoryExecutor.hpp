#ifndef AIKIDO_CONTROL_QUEUEDTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_QUEUEDTRAJECTORYEXECUTOR_HPP_

#include <queue>
#include <dart/dart.hpp>
#include "aikido/control/TrajectoryExecutor.hpp"

namespace aikido {
namespace control {

/// Wraps a TrajectoryExecutor to enable queuing trajectories for execution.
class QueuedTrajectoryExecutor : public TrajectoryExecutor
{
public:
  /// Constructor
  ///
  /// \param executor Underlying TrajectoryExecutor
  explicit QueuedTrajectoryExecutor(
      std::shared_ptr<TrajectoryExecutor> executor);

  virtual ~QueuedTrajectoryExecutor();

  // Documentation inherited.
  void validate(const trajectory::Trajectory* traj) override;

  /// Execute trajectory and set future upon completion. If another trajectory
  /// is already running, queue the trajectory for later execution. If executing
  /// a trajectory terminates in an error, all queued trajectories are aborted.
  ///
  /// \param traj Trajectory to be executed or queued.
  /// \return future<void> for trajectory execution. If trajectory terminates
  ///        before completion, future will be set to a runtime_error.
  /// \throws invalid_argument if traj is invalid.
  std::future<void> execute(
      const trajectory::ConstTrajectoryPtr& traj) override;

  // Documentation inherited.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  /// Abort the current trajectory, as well as all trajectories currently in the
  /// queue. Does NOT stop the trajectory that is currently executing if the
  /// underlying executor does not support it.
  void abort() override;

private:
  /// Underlying TrajectoryExecutor
  std::shared_ptr<TrajectoryExecutor> mExecutor;

  /// Whether a trajectory is currently being executed
  bool mInProgress;

  /// Future from wrapped executor
  std::future<void> mFuture;

  /// Queue of trajectories
  std::queue<trajectory::ConstTrajectoryPtr> mTrajectoryQueue;

  /// Queue of promises made by this to the client
  std::queue<std::shared_ptr<std::promise<void>>> mPromiseQueue;

  /// Manages access to mInProgress, mFuture, mTrajectoryQueue, mPromiseQueue
  std::mutex mMutex;
};

} // namespace control
} // namespace aikido

#endif // AIKIDO_CONTROL_QUEUEDTRAJECTORYEXECUTOR_HPP_
