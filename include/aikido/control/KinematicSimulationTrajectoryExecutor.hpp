#ifndef AIKIDO_CONTROL_KINEMATICSIMULATIONTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_KINEMATICSIMULATIONTRAJECTORYEXECUTOR_HPP_
#include "TrajectoryExecutor.hpp"
#include <aikido/trajectory/Trajectory.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

#include <future>
#include <mutex>
#include <condition_variable>

namespace aikido {
namespace control {

/// Executes trajectories in DART. This will spin a new thread
/// and simulate trajectories by setting dof positions
/// without running dynamic simulation.
class KinematicSimulationTrajectoryExecutor : public TrajectoryExecutor
{
public:
  /// Constructor.
  /// \param skeleton Skeleton to execute trajectories on.
  ///        All trajectories must have dofs only in this skeleton.
  /// \param executionCycle Sets the cycle period of the execution thread.
  template <typename Duration>
  KinematicSimulationTrajectoryExecutor(
    ::dart::dynamics::SkeletonPtr skeleton, 
    const Duration& executionCycle = std::chrono::milliseconds{1000});

  virtual ~KinematicSimulationTrajectoryExecutor();

  /// Execute traj and set future upon completion. 
  /// \param traj Trajectory to be executed. Its StateSpace should be a 
  ///        MetaStateSpace over MetaSkeleton, and the dofs in the metaskeleton 
  ///        should be all in skeleton passed to the constructor.
  /// \return future<void> for trajectory execution. If trajectory terminates
  ///        before completion, future will be set to a runtime_error.
  /// \throws invalid_argument if traj is invalid.
  std::future<void> execute(
    trajectory::TrajectoryPtr traj) override;

  // Documentation inherited.
  void step() override;

private:

  ::dart::dynamics::SkeletonPtr mSkeleton;
  std::unique_ptr<std::promise<void>> mPromise;
  trajectory::TrajectoryPtr mTraj; 
  
  /// step()'s trajectory execution cycle.
  const std::chrono::milliseconds mExecutionCycle;

  /// Time past since beginning of current trajectory's execution
  std::chrono::milliseconds mTimeSinceBeginning;

  /// Manages access on mTraj, mPromise, mInExecution
  std::mutex mMutex;
 
  bool mInExecution;

};

} // control
} // aikido

#endif
