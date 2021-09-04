#ifndef AIKIDO_CONTROL_JOINTCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_JOINTCOMMANDEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <set>

#include "aikido/common/pointers.hpp"
#include "aikido/control/Executor.hpp"

namespace aikido {
namespace control {

/// Abstract class for executing joint velocity commands
template <ExecutorType T>
class JointCommandExecutor : public Executor
{
public:
  /// Velocity-specific constructor.
  /// \param[in] joints Vector of joint names this Executor acts upon
  JointCommandExecutor(std::vector<std::string> joints) : Executor(T, joints)
  {
  }

  virtual ~JointCommandExecutor()
  {
  }

  /// Execute a Joint Command, setting future upon completion
  /// \note Future should return 0 on success or timeout.
  ///
  /// \param command Vector of joint commands, parallel with joints vector
  /// \param timeout How long until command expires
  virtual std::future<int> execute(
      const std::vector<double> command,
      const std::chrono::duration<double>& timeout)
      = 0;

  // Documentation inherited.
  virtual void step(
      const std::chrono::system_clock::time_point& timepoint) override = 0;

  /// Cancels the current command.
  virtual void cancel() = 0;
};

// Define common executor types
using VelocityExecutor = JointCommandExecutor<ExecutorType::VELOCITY>;
using PositionExecutor = JointCommandExecutor<ExecutorType::POSITION>;
using EffortExecutor = JointCommandExecutor<ExecutorType::EFFORT>;

} // namespace control
} // namespace aikido

#endif
