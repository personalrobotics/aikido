#ifndef AIKIDO_CONTROL_HANDSIMULATIONTRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_HANDSIMULATIONTRAJECTORYEXECUTOR_HPP_
#include <chrono>
#include "FingerSimulationStepExecutor.hpp"
#include <dart/collision/CollisionDetector.h>
#include <dart/collision/Option.h>
#include <dart/collision/CollisionGroup.h>
#include <dart/collision/CollisionFilter.h>
#include <dart/dynamics/dynamics.h>

namespace aikido {
namespace control {

/// Trajectory executor for BarretHand fingers. 
/// Assumes that fingers are underactuated: primal joint is actuated
/// and distal joint moves with certain mimic ratio (set to be Barret hand) 
/// until collision. 
class HandSimulationTrajectoryExecutor
{
public:
  /// Constructor.
  /// \param _fingers Fingers to be controlled by this Executor.
  ///        First 2 fingers should have 3 joints (spread, primal, distal)
  ///        Third finger should have 2 joints (primal, joint).
  /// \param _cyclePeriod
  /// \param _collisionDetector
  /// \param _collideWith
  /// \param _collisionOptions 
   HandSimulationTrajectoryExecutor(
    std::vector<::dart::dynamics::ChainPtr> _fingers,
    std::chrono::milliseconds _cyclePeriod,
    ::dart::collision::CollisionDetectorPtr _collisionDetector,
    ::dart::collision::CollisionGroupPtr _collideWith,
    ::dart::collision::Option _collisionOptions = ::dart::collision::Option(
      false, true, 1,
      std::make_shared<::dart::collision::BodyNodeCollisionFilter>()));

  /// Move the fingers until primal joints are set to _set_points. 
  /// If setpoints/spread are above/below dof limit, this will map to dof limit.
  /// \param _set_points End dof pose for primal joints.
  /// \param _spread Spread of two symmetric fingers
  /// \param _duratoin Max duration to execute the trajectory. 
  ///        The trajectory may terminate early upon collision.
  void execute(std::vector<double> _set_points, 
    double _spread, std::chrono::milliseconds _duration);

private:

  std::vector<::dart::dynamics::ChainPtr> mFingers;

  /// Contains primal and distal joints of all three fingers. 
  std::vector<::dart::dynamics::ChainPtr> mSubFingers;

  /// Executor for primal and distal joints. 
  std::vector<FingerSimulationStepExecutor> mFingerPrimalDofExecutors;

  ::dart::collision::CollisionDetectorPtr mCollisionDetector;
  ::dart::collision::CollisionGroupPtr mCollideWith;
  ::dart::collision::Option mCollisionOptions;

  std::vector<::dart::collision::CollisionGroupPtr> mPrimalCollisionGroups;
  std::vector<::dart::collision::CollisionGroupPtr> mDistalCollisionGroups;

  std::chrono::milliseconds mCyclePeriod;

};

} // control
} // aikido

#endif
