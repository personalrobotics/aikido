#ifndef AIKIDO_CONTROL_EXECUTOR_HPP_
#define AIKIDO_CONTROL_EXECUTOR_HPP_

#include <chrono>
#include <vector>
#include <set>

#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/pointers.hpp"
#include <dart/dart.hpp>

namespace aikido {
namespace control {

AIKIDO_DECLARE_POINTERS(Executor)

/// Type of executor
/// Can be used to determine if 2 executors make conflicting
/// demands of individual degrees of freedom (Dofs)
/// Can also be used to gracefully dynamic_cast
/// Roughly analogous to default ROS control types: 
/// https://wiki.ros.org/ros_control
/// The following updates the state of the DoF directly:
/// STATE
/// The following do not necessarily update the DoF's state directly,
/// but may require locking external resources:
/// POSITION - commands position to dofs
/// VELOCITY - commands velocity to dofs
/// EFFORT   - commands effort (i.e. torque) to dofs
/// TRAJECTORY - commands a trajectory to dofs
/// MODE - commands a hardware command mode (e.g. position/velocity/effort)
/// The following doesn't update the DoF directly:
/// READONLY
enum class ExecutorType
{
  STATE = -1,
  POSITION = 0,
  VELOCITY = 1,
  EFFORT = 2,
  TRAJECTORY = 3,
  MODE = 4,
  READONLY = 5
};

// Default rate for ExecutorThread to call step()
constexpr std::chrono::milliseconds defaultThreadRate{10};

/// Abstract class for executing commands on degrees of freedom
class Executor
{
public:
  /// Constructor.
  /// \param[in] types Set of controller types
  /// \param[in] dofs Vector of degree-of-freedom names this Executor acts upon
  /// \param[in] threadRate (Optional) How often to call step()
  Executor(
      const std::set<ExecutorType>& types,
      const std::vector<dart::dynamics::DegreeOfFreedom*>& dofs,
      const std::chrono::milliseconds threadRate = defaultThreadRate);

  Executor(
      const ExecutorType type,
      const std::vector<dart::dynamics::DegreeOfFreedom*>& dofs,
      std::chrono::milliseconds threadRate = defaultThreadRate);

  virtual ~Executor();

  /// Get all of this Executor's ExecutorTypes
  std::set<ExecutorType> getTypes() const;

  /// Get list of dofs needed by this Executor
  std::vector<dart::dynamics::DegreeOfFreedom*> getDofs() const;

  /// Step to a point in time.
  /// \note \c timepoint can be a time in the future to enable faster than
  /// real-time execution.
  ///
  /// \param timepoint Time to simulate to
  virtual void step(const std::chrono::system_clock::time_point& /* timepoint */) { /* Do Nothing */ }

  /// Start the underlying ExecutorThread
  void start();

  /// Stops the underlying ExecutorThread
  void stop();

  /// Lock the resources required by the DoFs
  ///
  /// \return True if locked successfully, false otherwise.
  bool registerDofs();

  /// Unlock any resources required by the DoFs
  void releaseDofs();

private:
  /// Call to spin first to pass current time to step
  /// Necessary for real-time execution via mThread.
  void spin()
  {
    if (mThread->isRunning())
    {
      step(std::chrono::system_clock::now());
    }
  }

  /// How often to run spin() on internal thread
  std::chrono::milliseconds mThreadRate;

  /// Whether this executor has a lock on its DoF resources
  bool mDofsRegistered{false};

  /// Manager for locking resources for degrees of freedom
  static std::unordered_map<ExecutorType, std::set<dart::dynamics::DegreeOfFreedom*>> mDofManager;

  /// Mutex to protects the DofManager
  static std::mutex mMutex;

  /// Executor thread calling step function
  std::unique_ptr<aikido::common::ExecutorThread> mThread;

protected:
  /// Vector of executor types
  std::set<ExecutorType> mTypes;

  /// Vector of dof names
  std::vector<dart::dynamics::DegreeOfFreedom*> mDofs;
};

} // namespace control
} // namespace aikido

#endif
