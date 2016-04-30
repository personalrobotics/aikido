#ifndef AIKIDO_CONTROL_KINEMATICSIMULATIONTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_KINEMATICSIMULATIONTRAJECTORYEXECUTOR_HPP_
#include "TrajectoryExecutor.hpp"
#include "../trajectory/Trajectory.hpp"
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"

#include <future>
#include <mutex>

namespace aikido {
namespace control {

class KinematicSimulationTrajectoryExecutor : public virtual TrajectoryExecutor
{
public:
  explicit KinematicSimulationTrajectoryExecutor(
    const ::dart::dynamics::SkeletonPtr& _skeleton);

  virtual ~KinematicSimulationTrajectoryExecutor();

  std::future<TrajectoryResultPtr> execute(
    trajectory::TrajectoryPtr _traj) override;

private:

  std::mutex mMutex;
  std::thread mThread;
  bool mRunning;
  ::dart::dynamics::SkeletonPtr mSkeleton;
  std::unique_ptr<std::promise<TrajectoryResultPtr>> mPromise;
  trajectory::TrajectoryPtr mTraj; 

  void spin(); 
};

} // control
} // aikido

#endif
