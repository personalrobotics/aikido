#ifndef AIKIDO_CONTROL_JOINTMODECOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_JOINTMODECOMMANDEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <set>

#include <dart/dart.hpp>

#include <hardware_interface/joint_mode_interface.h>

#include "aikido/common/pointers.hpp"
#include "aikido/control/Executor.hpp"
#include "aikido/control/util.hpp"

namespace aikido {
namespace control {

/// Abstract class for executing joint mode commands.
class JointModeCommandExecutor : public Executor
{
public:
  /// Constructor
  /// Documentation Inherited
  /// \param[in] otherTypes Other resources this executor needs beyond
  /// MODE
  JointModeCommandExecutor(
      const std::vector<dart::dynamics::DegreeOfFreedom*>& dofs,
      const std::set<ExecutorType> otherTypes = std::set<ExecutorType>(), // Rajat TODO: Remove this?
      const std::chrono::milliseconds threadRate = defaultThreadRate)
    : Executor(
        concatenateTypes(
            std::set<ExecutorType>{ExecutorType::MODE}, otherTypes),
        dofs,
        threadRate)
  {
  }

  virtual ~JointModeCommandExecutor()
  {
  }

  /// Execute a Joint Mode Command, setting future upon completion
  /// \note Future should return 0 on success or timeout.
  ///
  /// \param command Vector of joint mode commands, parallel with joints vector
  /// \param timeout How long until command expires
  virtual std::future<int> execute(
      const std::vector<hardware_interface::JointCommandModes>& command_modes,
      const std::chrono::duration<double>& timeout,
      const std::chrono::system_clock::time_point& timepoint)
      = 0;
  virtual std::future<int> execute(
      const std::vector<hardware_interface::JointCommandModes>& command_modes,
      const std::chrono::duration<double>& timeout)
  {
    return this->execute(command, timeout, std::chrono::system_clock::now());
  }

  // Documentation inherited.
  virtual void step(
      const std::chrono::system_clock::time_point& timepoint) override = 0;

  /// Cancels the current command.
  virtual void cancel() = 0;
};

} // namespace control
} // namespace aikido

#endif
