#ifndef AIKIDO_CONTROL_KINEMATICSIMULATIONTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_KINEMATICSIMULATIONTRAJECTORYEXECUTOR_HPP_
#include "TrajectoryExecutor.hpp"
#include "../trajectory/Trajectory.hpp"
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"

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
  /// \param _skeleton Skeleton to execute trajectories on.
  ///        All trajectories must have dofs only in this skeleton.
  /// \param _period Sets the cycle period of the execution thread.
  KinematicSimulationTrajectoryExecutor(
    ::dart::dynamics::SkeletonPtr _skeleton, 
    std::chrono::milliseconds _period);

  virtual ~KinematicSimulationTrajectoryExecutor();

  std::future<void> execute(
    trajectory::TrajectoryPtr _traj) override;

private:

  ::dart::dynamics::SkeletonPtr mSkeleton;
  std::unique_ptr<std::promise<void>> mPromise;
  trajectory::TrajectoryPtr mTraj; 
  
  // spin()'s trajectory execution cycle.
  std::chrono::milliseconds mPeriod;

  // Blocks spin() until execute(...) is called; paired with mSpinLock.
  std::condition_variable mCv;

   // Lock for keeping spin thread alive and executing a trajectory. 
   // Manages access on mTraj, mPromise, mRunning
  std::mutex mSpinLock;

  // Thread for spin().
  std::thread mThread;
  
  // Flag for killing spin thread. 
  bool mRunning;

  // Simulates mTraj. To be executed on a separate thread.
  void spin(); 
};

} // control
} // aikido

#endif
