#ifndef AIKIDO_CONTROL_QUEUEDTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_QUEUEDTRAJECTORYEXECUTOR_HPP_

#include <queue>
#include <dart/dart.hpp>
#include <aikido/control/TrajectoryExecutor.hpp>

namespace aikido {
namespace control {

/// Wraps a TrajectoryExecutor to enable queuing trajectories for execution.
class QueuedTrajectoryExecutor : public TrajectoryExecutor
{
public:
  /// Constructor
  /// \param executor Underlying TrajectoryExecutor
  explicit QueuedTrajectoryExecutor(
      std::shared_ptr<TrajectoryExecutor> executor);

  virtual ~QueuedTrajectoryExecutor();

  // Documentation inherited.
  void validate(trajectory::TrajectoryPtr traj) override;

  /// Execute trajectory and set future upon completion. If another trajectory
  /// is already running, queue the trajectory for later execution.
  /// \param traj Trajectory to be executed or queued.
  /// \return future<void> for trajectory execution. If trajectory terminates
  ///        before completion, future will be set to a runtime_error.
  /// \throws invalid_argument if traj is invalid.
  std::future<void> execute(trajectory::TrajectoryPtr traj) override;

  void step() override;

private:
  /// Underlying TrajectoryExecutor
  std::shared_ptr<TrajectoryExecutor> mExecutor;

  /// Whether a trajectory is currently being executed
  bool mInProgress;

  /// Future from wrapped executor
  std::future<void> mFuture;

  /// Queue of trajectories
  std::queue<trajectory::TrajectoryPtr> mTrajectoryQueue;

  /// Queue of promises
  std::queue<std::shared_ptr<std::promise<void>>> mPromiseQueue;

  /// Manages access on mInProgress, mFuture, mTrajectoryQueue, mPromiseQueue
  std::mutex mMutex;
};

} // namespace control
} // namespace aikido

#endif // AIKIDO_CONTROL_QUEUEDTRAJECTORYEXECUTOR_HPP_
