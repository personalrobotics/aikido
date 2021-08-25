#ifndef AIKIDO_CONTROL_EXECUTOR_HPP_
#define AIKIDO_CONTROL_EXECUTOR_HPP_

#include <chrono>
#include <future>
#include <set>

#include "aikido/common/pointers.hpp"
#include "aikido/common/ExecutorThread.hpp"

namespace aikido {
namespace control {

AIKIDO_DECLARE_POINTERS(Executor)

enum class ExecutorType
{
  kSTATE = -1,
  kPOSITION = 0,
  kVELOCITY = 1,
  kEFFORT = 2,
  kTRAJECTORY = 3,
  kOTHER = 4
};

/// Abstract class for executing commands
class Executor
{
public:
  // Parameter defaults
  constexpr static std::chrono::milliseconds cDefaultThreadRate{10};

  virtual ~Executor() = default;

  /// Constructor.
  /// \param[in] types Vector of controller types
  /// \param[in] joints Vector of joint names this Executor acts upon
  /// \param[in] threadRate (Optional) How often to call step()
  Executor(std::vector<ExecutorType> types, std::vector<std::string> joints, std::chrono::milliseconds threadRate = cDefaultThreadRate)
    : mTypes(types)
    , mJoints(joints)
  {
    mThread = std::make_unique<aikido::common::ExecutorThread>(
      std::bind(&Executor::step, this, std::chrono::system_clock::now()), threadRate);
  }

  Executor(ExecutorType type, std::vector<std::string> joints, std::chrono::milliseconds threadRate = cDefaultThreadRate)
    : Executor(std::vector<ExecutorType>{type}, joints, threadRate) {}

  /// Get primary ExecutorType
  ExecutorType getType() { return mTypes[0]; }

  // Get all ExecutorTypes
  std::vector<ExecutorType> getTypes() { return mTypes; }

  // Get Executor Joint Lists
  std::vector<std::string> getJoints() { return mJoints; }

  /// Step to a point in time.
  /// \note \c timepoint can be a time in the future to enable faster than
  /// real-time execution.
  ///
  /// \param timepoint Time to simulate to
  virtual void step(const std::chrono::system_clock::time_point& timepoint) = 0;

  /// Stops the underlying ExecutorThread
  void stop() { mThread->stop(); }

protected:
  /// Executor thread calling step function
  std::unique_ptr<aikido::common::ExecutorThread> mThread;

  /// Vector of executor types
  std::vector<ExecutorType> mTypes;

  /// Vector of joint names
  std::vector<std::string> mJoints;
};

} // namespace control
} // namespace aikido

#endif
