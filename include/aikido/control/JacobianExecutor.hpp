#ifndef AIKIDO_CONTROL_JACOBIANEXECUTOR_HPP_
#define AIKIDO_CONTROL_JACOBIANEXECUTOR_HPP_

#include <future>
#include <mutex>

#include <dart/dynamics/Skeleton.hpp>

#include "aikido/control/Executor.hpp"
#include "aikido/control/JointCommandExecutor.hpp"
#include "aikido/control/util.hpp"

namespace aikido {
namespace control {

/// Executes end-effector SE3 command.
/// Uses local Jacobian to translate to joint command.
///
template <ExecutorType T>
class JacobianExecutor : public JointCommandExecutor<T>
{
public:
  static constexpr size_t SE3_SIZE = 6;
  static constexpr double DEFAULT_LAMBDA = 1E-1;

  /// Constructor.
  ///
  /// \param eeNode End effector body node.
  /// \param lambda Damped Jacobian pseudo-inverse calculation: (JtJ + lambda^2
  /// I)^{-1} \param executor Base joint executor for converted joint commands.
  ///                 Default: KinematicSimulationJointCommandExecutor
  explicit JacobianExecutor(
      ::dart::dynamics::BodyNode* eeNode,
      std::shared_ptr<JointCommandExecutor<T>> executor = nullptr,
      double lambda = DEFAULT_LAMBDA);

  virtual ~JacobianExecutor();

  /// Execute an SE3 Command, setting future upon completion
  /// \note Future should return 0 on success or timeout.
  ///
  /// \param command SE3 Command Vector (6d)
  /// \param timeout How long until command expires
  virtual std::future<int> execute(
      const Eigen::Vector6d command,
      const std::chrono::duration<double>& timeout
      = std::chrono::duration<double>(1))
  {
    return this->execute(command, timeout, std::chrono::system_clock::now());
  }
  virtual std::future<int> execute(
      const Eigen::Vector6d command,
      const std::chrono::duration<double>& timeout,
      const std::chrono::system_clock::time_point& timepoint);

  /// \copydoc JointCommandExecutor::execute()
  ///
  /// Cancels all SE3 Commands
  /// Calls the underlying JointCommandExecutor.
  virtual std::future<int> execute(
      const std::vector<double>& command,
      const std::chrono::duration<double>& timeout
      = std::chrono::duration<double>(1))
  {
    return this->execute(command, timeout, std::chrono::system_clock::now());
  }
  virtual std::future<int> execute(
      const std::vector<double>& command,
      const std::chrono::duration<double>& timeout,
      const std::chrono::system_clock::time_point& timepoint);

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
  std::vector<double> SE3ToJoint(const Eigen::Vector6d command);

  /// End Effector Body Node Name
  ::dart::dynamics::BodyNode* mEENode;

  /// Underlying Joint Command Executor
  std::shared_ptr<JointCommandExecutor<T>> mExecutor;

  /// Command being executed (6d)
  Eigen::Vector6d mCommand;

  /// Jacobian damping factor
  double mLambda;

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

// Define common template arguments
using JacobianVelocityExecutor = JacobianExecutor<ExecutorType::VELOCITY>;
using JacobianEffortExecutor = JacobianExecutor<ExecutorType::EFFORT>;
// Note: No position, as jacobian should only be used with derivatives.

// Define constexprs (required for linking)
template <ExecutorType T>
constexpr size_t JacobianExecutor<T>::SE3_SIZE;
template <ExecutorType T>
constexpr double JacobianExecutor<T>::DEFAULT_LAMBDA;

} // namespace control
} // namespace aikido

#include "detail/JacobianExecutor-impl.hpp"

#endif
