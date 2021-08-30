#ifndef AIKIDO_CONTROL_POSITIONEXECUTOR_HPP_
#define AIKIDO_CONTROL_POSITIONEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <set>

#include "aikido/common/pointers.hpp"
#include "aikido/control/Executor.hpp"

namespace aikido {
namespace control {

AIKIDO_DECLARE_POINTERS(PositionExecutor)

/// Abstract class for executing joint position commands
class PositionExecutor : public Executor
{
public:
  /// Position-specific constructor.
  /// \param[in] joints Vector of joint names this Executor acts upon
  PositionExecutor(std::vector<std::string> joints)
    : Executor(ExecutorType::kPOSITION, joints)
  {
  }

  /// Execute a Joint Position Command, setting future upon completion
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
