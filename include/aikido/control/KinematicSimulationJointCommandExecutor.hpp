#ifndef AIKIDO_CONTROL_KINEMATICSIMULATIONJOINTCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_KINEMATICSIMULATIONJOINTCOMMANDEXECUTOR_HPP_

#include <future>
#include <mutex>

#include <dart/dynamics/Skeleton.hpp>

#include "aikido/control/JointCommandExecutor.hpp"
#include "aikido/control/util.hpp"

namespace aikido {
namespace control {

/// Executes joint command in DART. This simulates by setting
/// interpolated DOF positions, without running dynamic simulation.
template <ExecutorType T>
class KinematicSimulationJointCommandExecutor : public JointCommandExecutor<T>
{
public:
  /// Constructor.
  ///
  /// \param metaskeleton MetaSkeleton to execute commands on.
  explicit KinematicSimulationJointCommandExecutor(
      ::dart::dynamics::MetaSkeletonPtr metaskeleton);

  virtual ~KinematicSimulationJointCommandExecutor();

  // Documentation inherited
  virtual std::future<int> execute(
      const std::vector<double>& command,
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

// Define common template arguments
using KinematicSimulationPositionExecutor
    = KinematicSimulationJointCommandExecutor<ExecutorType::POSITION>;
using KinematicSimulationVelocityExecutor
    = KinematicSimulationJointCommandExecutor<ExecutorType::VELOCITY>;
// Note: No effort simulation without dynamics

} // namespace control
} // namespace aikido

#include "detail/KinematicSimulationJointCommandExecutor-impl.hpp"

#endif
