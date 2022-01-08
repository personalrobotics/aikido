#ifndef AIKIDO_CONTROL_KINEMATICSIMULATIONTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_KINEMATICSIMULATIONTRAJECTORYEXECUTOR_HPP_

#include <future>
#include <mutex>

#include <dart/dynamics/Skeleton.hpp>

#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace control {

/// Executes trajectories in DART. This simulates trajectories by setting
/// interpolated DOF positions, without running dynamic simulation.
class KinematicSimulationTrajectoryExecutor : public TrajectoryExecutor
{
public:
  /// Constructor.
  ///
  /// \param metaskeleton MetaSkeleton to execute trajectories on.
  ///        All trajectories must have dofs only in this skeleton.
  explicit KinematicSimulationTrajectoryExecutor(
      ::dart::dynamics::MetaSkeletonPtr metaskeleton);

  virtual ~KinematicSimulationTrajectoryExecutor();

  // Documentation inherited.
  void validate(const trajectory::Trajectory* traj) override;

  /// Execute traj and set future upon completion.
  ///
  /// \param traj Trajectory to be executed.
  ///        Its StateSpace should be a MetaSkeletonStateSpace, where the dofs
  ///        are all in the skeleton passed to this constructor.
  /// \return future<void> for trajectory execution. If trajectory terminates
  ///        before completion, future will be set to a runtime_error.
  /// \throws invalid_argument if traj is invalid.
  std::future<void> execute(
      const trajectory::ConstTrajectoryPtr& traj) override;

  /// \copydoc TrajectoryExecutor::step()
  ///
  /// If multiple threads are accessing this function or the skeleton associated
  /// with this executor, it is necessary to lock the skeleton before
  /// calling this method.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  /// Cancels the current trajectory.
  void cancel() override;

private:
  /// MetaSkeleton to execute trajectories on
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;

  /// Trajectory being executed
  trajectory::ConstTrajectoryPtr mTraj;

  /// Trajectory's MetaSkeletonStateSpace
  statespace::dart::ConstMetaSkeletonStateSpacePtr mStateSpace;

  /// Whether a trajectory is being executed
  bool mInProgress;

  /// Promise whose future is returned by execute()
  std::unique_ptr<std::promise<void>> mPromise;

  /// Manages access to mTraj, mInProgress, mPromise
  mutable std::mutex mMutex;
};

} // namespace control
} // namespace aikido

#endif
