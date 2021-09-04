#ifndef AIKIDO_CONTROL_EXECUTOR_HPP_
#define AIKIDO_CONTROL_EXECUTOR_HPP_

#include <chrono>
#include <future>
#include <set>
#include <vector>

#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/pointers.hpp"

namespace aikido {
namespace control {

AIKIDO_DECLARE_POINTERS(Executor)

enum class ExecutorType
{
  STATE = -1,
  POSITION = 0,
  VELOCITY = 1,
  EFFORT = 2,
  TRAJECTORY = 3,
  OTHER = 4
};

// Parameter defaults
constexpr std::chrono::milliseconds defaultThreadRate{10};

/// Abstract class for executing commands
class Executor
{
public:
  virtual ~Executor()
  {
    stop();
  }

  /// Constructor.
  /// \param[in] types Vector of controller types
  /// \param[in] joints Vector of joint names this Executor acts upon
  /// \param[in] threadRate (Optional) How often to call step()
  Executor(
      const std::vector<ExecutorType> types,
      const std::vector<std::string> joints,
      std::chrono::milliseconds threadRate = defaultThreadRate)
    : mThreadRate(threadRate), mThread(nullptr), mTypes(types), mJoints(joints)
  {
  }

  Executor(
      const ExecutorType type,
      const std::vector<std::string> joints,
      std::chrono::milliseconds threadRate = defaultThreadRate)
    : Executor(std::vector<ExecutorType>{type}, joints, threadRate)
  {
  }

  /// Get primary ExecutorType
  ExecutorType getType()
  {
    return mTypes[0];
  }

  /// Get all of this Executor's ExecutorTypes
  std::vector<ExecutorType> getTypes()
  {
    return mTypes;
  }

  /// Get list of joints needed by this Executor
  std::vector<std::string> getJoints()
  {
    return mJoints;
  }

  /// Step to a point in time.
  /// \note \c timepoint can be a time in the future to enable faster than
  /// real-time execution.
  ///
  /// \param timepoint Time to simulate to
  virtual void step(const std::chrono::system_clock::time_point& timepoint) = 0;

  /// Start the underlying ExecutorThread
  void start()
  {
    if (!mThread)
    {
      mThread = std::make_unique<aikido::common::ExecutorThread>(
          std::bind(&Executor::spin, this), mThreadRate);
    }
  }

  /// Stops the underlying ExecutorThread
  void stop()
  {
    if (mThread)
    {
      mThread->stop();
      mThread.reset();
    }
  }

private:
  /// Call to spin first to pass current time to step
  void spin()
  {
    if (mThread->isRunning())
    {
      step(std::chrono::system_clock::now());
    }
  }

  std::chrono::milliseconds mThreadRate;

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
