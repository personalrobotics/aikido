#ifndef AIKIDO_CONTROL_PIDTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_PIDTRAJECTORYEXECUTOR_HPP_

#include <future>
#include <mutex>
#include <Eigen/Dense>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace control {

struct PidGains
{
  /// Constructor
  PidGains(
      Eigen::VectorXd p,
      Eigen::VectorXd i,
      Eigen::VectorXd d,
      Eigen::VectorXd i_max,
      Eigen::VectorXd i_min);

  /// Proportional gains
  Eigen::VectorXd mProportionalGains;

  /// Integral gains
  Eigen::VectorXd mIntegralGains;

  /// Derivative gains
  Eigen::VectorXd mDerivativeGains;

  /// Maximum integral term value
  Eigen::VectorXd mIntegralMax;

  /// Minimum integral term value
  Eigen::VectorXd mIntegralMin;
};

/// Executes trajectories in DART with PID control.
class PidTrajectoryExecutor : public TrajectoryExecutor
{
public:
  /// Constructor.
  ///
  /// \param skeleton Skeleton to execute trajectories on.
  ///        All trajectories must have dofs only in this skeleton.
  /// \param world DART World to simulate during each call to \c step.
  ///        Skeleton must be in this world.
  /// \param gains PID gains.
  explicit PidTrajectoryExecutor(
      ::dart::dynamics::SkeletonPtr skeleton,
      ::dart::simulation::WorldPtr world,
      PidGains gains);

  virtual ~PidTrajectoryExecutor();

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

  /// Aborts the current trajectory.
  void abort() override;

private:
  /// Time of previous call to \c step.
  /// \note If \c step has not yet been called, this is the time that \c execute
  /// was called.
  std::chrono::system_clock::time_point mTimeOfPreviousCall;

  /// Skeleton to execute trajectories on
  ::dart::dynamics::SkeletonPtr mSkeleton;

  /// DART World for simulation
  ::dart::simulation::WorldPtr mWorld;

  /// PID gains to use at each step
  PidGains mGains;

  /// Proportional error
  Eigen::VectorXd mProportionalError;

  /// Integral error
  Eigen::VectorXd mIntegralError;

  /// Derivative error
  Eigen::VectorXd mDerivativeError;

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

#endif // AIKIDO_CONTROL_PIDTRAJECTORYEXECUTOR_HPP_
