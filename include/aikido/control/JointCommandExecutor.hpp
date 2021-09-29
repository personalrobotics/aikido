#ifndef AIKIDO_CONTROL_JOINTCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_JOINTCOMMANDEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <set>

#include <dart/dart.hpp>

#include "aikido/common/pointers.hpp"
#include "aikido/control/Executor.hpp"
#include "aikido/control/util.hpp"

namespace aikido {
namespace control {

/// Abstract class for executing a command of a single type
/// on a group of joints.
/// \tparam T The primary and only type of this executor.
template <ExecutorType T>
class JointCommandExecutor : public Executor
{
public:
  /// Constructor
  /// Documentation inherited.
  /// \param[in] otherTypes Other resources this executor needs beyond the
  /// primary
  JointCommandExecutor(
      const std::vector<dart::dynamics::DegreeOfFreedom*> dofs,
      const std::set<ExecutorType> otherTypes = std::set<ExecutorType>(),
      const std::chrono::milliseconds threadRate = defaultThreadRate)
    : Executor(
        concatenateTypes(std::set<ExecutorType>{T}, otherTypes),
        dofs,
        threadRate)
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
      const std::vector<double>& command,
      const std::chrono::duration<double>& timeout)
      = 0;

  // Documentation inherited.
  virtual void step(
      const std::chrono::system_clock::time_point& timepoint) override = 0;

  /// Cancels the current command.
  virtual void cancel() = 0;
};

// Define common executor types
using PositionExecutor = JointCommandExecutor<ExecutorType::POSITION>;
using VelocityExecutor = JointCommandExecutor<ExecutorType::VELOCITY>;
using EffortExecutor = JointCommandExecutor<ExecutorType::EFFORT>;

} // namespace control
} // namespace aikido

#endif
