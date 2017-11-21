#ifndef AIKIDO_CONTROL_INSTANTANEOUSTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_INSTANTANEOUSTRAJECTORYEXECUTOR_HPP_

#include <future>
#include <mutex>

#include <dart/dynamics/Skeleton.hpp>
#include <aikido/control/TrajectoryExecutor.hpp>
#include <aikido/trajectory/Trajectory.hpp>

namespace aikido {
namespace control {

/// Instantaneously executes a trajectory in simulation by setting DOF positions
/// to the end of the trajectory.
class InstantaneousTrajectoryExecutor : public TrajectoryExecutor
{
public:
  /// Constructor.
  /// \param skeleton Skeleton to execute trajectories on.
  ///        All trajectories must have dofs only in this skeleton.
  InstantaneousTrajectoryExecutor(::dart::dynamics::SkeletonPtr skeleton);

  virtual ~InstantaneousTrajectoryExecutor();

  // Documentation inherited.
  void validate(trajectory::TrajectoryPtr traj) override;

  /// Instantaneously execute traj and set future upon completion.
  /// \param traj Trajectory to be executed. Its StateSpace should be a
  ///        MetaStateSpace over MetaSkeleton, and the dofs in the metaskeleton
  ///        should be all in skeleton passed to the constructor.
  /// \return future<void> for trajectory execution. If trajectory terminates
  ///        before completion, future will be set to a runtime_error.
  /// \throws invalid_argument if traj is invalid.
  std::future<void> execute(trajectory::TrajectoryPtr traj) override;

  // Do nothing.
  void step() override;

private:
  ::dart::dynamics::SkeletonPtr mSkeleton;

  std::unique_ptr<std::promise<void>> mPromise;

  /// Manages access on mPromise
  std::mutex mMutex;
};

} // namespace control
} // namespace aikido

#endif
