#ifndef AIKIDO_CONTROL_INSTANTANEOUSTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_INSTANTANEOUSTRAJECTORYEXECUTOR_HPP_

#include <future>
#include <mutex>
#include <dart/dynamics/Skeleton.hpp>
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace control {

/// Instantaneously executes a trajectory in simulation by setting DOF positions
/// to the end of the trajectory.
class InstantaneousTrajectoryExecutor : public TrajectoryExecutor
{
public:
  /// Constructor.
  ///
  /// \param skeleton Skeleton to execute trajectories on.
  ///        All trajectories must have dofs only in this skeleton.
  explicit InstantaneousTrajectoryExecutor(
      ::dart::dynamics::SkeletonPtr skeleton);

  virtual ~InstantaneousTrajectoryExecutor();

  // Documentation inherited.
  void validate(const trajectory::Trajectory* traj) override;

  /// Instantaneously execute traj and set future upon completion.
  ///
  /// \param traj Trajectory to be executed.
  ///        Its StateSpace should be a MetaSkeletonStateSpace, where the dofs
  ///        are all in the skeleton passed to this constructor.
  /// \return future<void> for trajectory execution. If trajectory terminates
  ///        before completion, future will be set to a runtime_error.
  /// \throws invalid_argument if traj is invalid.
  std::future<void> execute(
      const trajectory::ConstTrajectoryPtr& traj) override;

  // Do nothing.
  void step(
      const std::chrono::system_clock::time_point& /*timepoint*/) override;

  // Do nothing.
  void cancel() override;

private:
  /// Skeleton to execute trajectories on
  ::dart::dynamics::SkeletonPtr mSkeleton;

  /// Promise whose future is returned by execute()
  std::unique_ptr<std::promise<void>> mPromise;

  /// Manages access to mPromise
  mutable std::mutex mMutex;
};

} // namespace control
} // namespace aikido

#endif
