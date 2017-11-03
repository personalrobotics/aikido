#ifndef AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_

#include <atomic>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <aikido/trajectory/Trajectory.hpp>
#include "TrajectoryResult.hpp"

namespace aikido {
namespace control {

class TrajectoryExecutor
{
public:
  virtual ~TrajectoryExecutor();

  /// Execute traj and set future upon completion.
  /// \param traj Trajectory to be executed.
  /// \param skip Whether to skip to the end of the trajectory (only used by
  ///        simulation trajectory executors)
  virtual std::future<void> execute(
      trajectory::TrajectoryPtr _traj, bool _skip = false)
      = 0;

  /// Executes one step.
  virtual void step() = 0;

  /// Queue a trajectory for future execution. If there are no trajectories on
  /// the queue or currently being executed, the trajectory should be executed
  /// immediately.
  /// \param traj Trajectory to queue and execute
  void queue(trajectory::TrajectoryPtr _traj);

  /// Toggle whether trajectories should be automatically dequeued and executed.
  /// \param flag Whether to automatically execute trajectories from the queue
  void setExecuteFromQueue(bool flag);

  /// Block until all trajectories in the queue have been executed.
  void executeAllFromQueue();

protected:
  TrajectoryExecutor();

  /// Thread target for automatically checking queue and executing trajectories
  void executeFromQueue();

  /// Queue of trajectories
  std::queue<trajectory::TrajectoryPtr> mTrajectoryQueue;

  /// Manages access to mTrajectoryQueue
  mutable std::mutex mTrajectoryQueueMutex;

  /// Whether trajectories are automatically dequeued and executed
  std::atomic_bool mRunning;

  /// Whether the thread should stop running when the queue is empty
  std::atomic_bool mReturnWhenEmpty;

  std::thread mThread;
};

using TrajectoryExecutorPtr = std::shared_ptr<TrajectoryExecutor>;

} // namespace control
} // namespace aikido

#endif
