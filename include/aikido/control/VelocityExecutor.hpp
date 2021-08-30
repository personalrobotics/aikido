#ifndef AIKIDO_CONTROL_VELOCITYEXECUTOR_HPP_
#define AIKIDO_CONTROL_VELOCITYEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <set>

#include "aikido/common/pointers.hpp"
#include "aikido/control/Executor.hpp"

namespace aikido {
namespace control {

AIKIDO_DECLARE_POINTERS(VelocityExecutor)

/// Abstract class for executing joint velocity commands
class VelocityExecutor : public Executor
{
public:
  /// Velocity-specific constructor.
  /// \param[in] joints Vector of joint names this Executor acts upon
  VelocityExecutor(std::vector<std::string> joints)
    : Executor(ExecutorType::kVELOCITY, joints)
  {
  }

  /// Execute a Joint Velocity Command, setting future upon completion
  /// \note Future should return 0 on success or timeout.
  ///
  /// \param command Vector of joint commands, parallel with joints vector
  /// \param timeout How long until command expires
  virtual std::future<int> execute(
      const std::vector<double> command,
      const std::chrono::duration<double>& timeout)
      = 0;

  /// Cancels the current command.
  virtual void cancel();
};

} // namespace control
} // namespace aikido

#endif
