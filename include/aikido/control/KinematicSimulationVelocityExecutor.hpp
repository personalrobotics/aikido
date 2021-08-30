#ifndef AIKIDO_CONTROL_KINEMATICSIMULATIONVELOCITYEXECUTOR_HPP_
#define AIKIDO_CONTROL_KINEMATICSIMULATIONVELOCITYEXECUTOR_HPP_

#include <future>
#include <mutex>

#include <dart/dynamics/Skeleton.hpp>

#include "aikido/control/VelocityExecutor.hpp"

namespace aikido {
namespace control {

/// Executes joint velocity command in DART. This simulates by setting
/// interpolated DOF positions, without running dynamic simulation.
class KinematicSimulationVelocityExecutor : public VelocityExecutor
{
public:
  /// Constructor.
  ///
  /// \param skeleton Skeleton to execute commands on.
  ///        All degrees of freedom are assumed to be commanded.
  explicit KinematicSimulationVelocityExecutor(
      ::dart::dynamics::SkeletonPtr skeleton);

  virtual ~KinematicSimulationVelocityExecutor();

  /// Documentation inherited
  virtual std::future<int> execute(
      const std::vector<double> command,
      const std::chrono::duration<double>& timeout
      = std::chrono::duration<double>(1)) override;

  /// \copydoc Executor::step()
  ///
  /// If multiple threads are accessing this function or the skeleton associated
  /// with this executor, it is necessary to lock the skeleton before
  /// calling this method.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  /// Cancels the current command.
  void cancel() override;

private:
  /// Skeleton to execute trajectories on
  ::dart::dynamics::SkeletonPtr mSkeleton;

  /// Command being executed
  std::vector<double> mCommand;

  /// The controlled subset of mSkeleton for the currently executing command.
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;

  /// Whether a command is being executed
  bool mInProgress;

  /// Promise whose future is returned by execute()
  std::unique_ptr<std::promise<int>> mPromise;

  /// Manages access to mCommand, mInProgress, mPromise
  mutable std::mutex mMutex;

  /// Time of position command start (for interpolation)
  std::chrono::system_clock::time_point mExecutionStartTime;

  /// Position at start of command (for interpolation)
  std::vector<double> mStartPosition;

  /// Velocity timeout
  std::chrono::duration<double> mTimeout;
};

} // namespace control
} // namespace aikido

#endif
