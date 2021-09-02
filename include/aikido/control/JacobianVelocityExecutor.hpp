#ifndef AIKIDO_CONTROL_JACOBIANVELOCITYEXECUTOR_HPP_
#define AIKIDO_CONTROL_JACOBIANVELOCITYEXECUTOR_HPP_

#include <future>
#include <mutex>

#include <dart/dynamics/Skeleton.hpp>

#include "aikido/control/Executor.hpp"
#include "aikido/control/VelocityExecutor.hpp"

namespace aikido {
namespace control {

/// Executes end-effector SE3 velocity command.
/// Uses local Jacobian to translate to joint velocity command.
class JacobianVelocityExecutor : public Executor
{
public:
  /// Constructor.
  ///
  /// \param skeleton Skeleton to execute commands on.
  ///        All degrees of freedom are assumed to be commanded.
  ///        Must be externally updated with joint state.
  /// \param eeName End effector body node name.
  /// \param executor Base joint executor for converted joint commands.
  explicit JacobianVelocityExecutor(
      ::dart::dynamics::SkeletonPtr skeleton,
      std::string eeName,
      VelocityExecutorPtr executor);

  virtual ~JacobianVelocityExecutor();

  /// Execute an SE3 Velocity Command, setting future upon completion
  /// \note Future should return 0 on success or timeout.
  ///
  /// \param command SE3 Command Vector (6d)
  /// \param timeout How long until command expires
  virtual std::future<int> execute(
      const std::vector<double> command,
      const std::chrono::duration<double>& timeout
      = std::chrono::duration<double>(1));

  /// \copydoc Executor::step()
  ///
  /// If multiple threads are accessing this function or the skeleton associated
  /// with this executor, it is necessary to lock the skeleton before
  /// calling this method.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  /// Cancels the current command.
  void cancel();

private:
  /// Convert SE3 command to joint command
  std::vector<double> SE3ToJoint(std::vector<double> cmd);

  /// Skeleton to execute trajectories on
  ::dart::dynamics::SkeletonPtr mSkeleton;

  /// End Effector Body Node Name
  std::string mEEName;

  /// Underlying Velocity Executor
  VelocityExecutorPtr mExecutor;

  /// Command being executed (6d)
  std::vector<double> mCommand;

  /// Whether a command is being executed
  bool mInProgress;

  /// Promise whose future is returned by execute()
  std::unique_ptr<std::promise<int>> mPromise;

  /// Manages access to mCommand, mInProgress, mPromise
  mutable std::mutex mMutex;

  /// Time of command start
  std::chrono::system_clock::time_point mExecutionStartTime;

  /// Velocity timeout
  std::chrono::duration<double> mTimeout;

  /// Future for underlying executor call
  std::future<int> mFuture;
};

} // namespace control
} // namespace aikido

#endif
