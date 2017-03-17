#ifndef AIKIDO_CONTROL_BARRETFINGERPOSITIONCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETFINGERPOSITIONCOMMANDEXECUTOR_HPP_
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <future>
#include <mutex>
#include <condition_variable>

namespace aikido {
namespace control {

/// This executor mimics the behavior of BarretFinger. 
/// It moves a finger to a desired point; it may stop early if 
/// joint limit is reached or collision is detected.
/// Only the proximal joint is actuated; the distal joint moves with mimic ratio.
/// When collision is detected on the distal link, the finger stops.
/// When collision is detected on the proximal link, the distal link moves
/// until it reaches joint limit or until distal collision is detected.
class BarrettFingerPositionCommandExecutor
{
public:
  /// Constructor.
  /// \param _finger Finger to be controlled by this Executor.
  /// \param _proximal Index of proximal dof
  /// \param _distal Index of distal dof
  /// \param _collisionDetector CollisionDetector for detecting self collision
  ///        and collision with objects. 
  /// \param _collisionOptions Default is (enableContact=false, binaryCheck=true,
  ///        maxNumContacts = 1.) 
  ///        See dart/collison/Option.h for more information 
  BarrettFingerPositionCommandExecutor(
    ::dart::dynamics::ChainPtr _finger, int _proximal, int _distal,
    ::dart::collision::CollisionDetectorPtr _collisionDetector,
    ::dart::collision::CollisionOption _collisionOptions
      = ::dart::collision::CollisionOption(false, 1));

  /// Sets variables to move the finger joints.
  /// proximal dof moves to _goalPosition, joint limit, or until collision.
  /// Step method should be called multiple times until future returns.
  /// Distal dof follows with mimic ratio. 
  /// \param _goalPosition Desired angle of proximal joint.
  /// \param _collideWith CollisionGroup to check collision with fingers.
  /// \return future Becomes available when the execution completes.
  std::future<void> execute(double _goalPosition,
    ::dart::collision::CollisionGroupPtr _collideWith);

  /// Returns mimic ratio, i.e. how much the distal joint moves relative to 
  /// the proximal joint. The joint movements follow
  /// this ratio only when both joints are moving.
  /// \return mimic ratio.
  static double getMimicRatio();

  /// Moves the joints of the finger by dofVelocity*_timeSIncePreviousCall
  /// until execute's goalPosition by primary dof or collision is detected.
  /// If proximal link is in collision, distal link moves until 
  /// mimicRatio*goalPosition. If distal link is in collision, execution stops.
  /// If multiple threads are accessing this function or skeleton associated
  /// with this executor, it is necessary to lock the skeleton before
  /// calling this method.
  /// \param _timeSincePreviousCall Time since previous call. 
  void step(double _timeSincePreviousCall);

private: 
  constexpr static double kMimicRatio = 0.333; 
  constexpr static double kProximalVelocity = 0.01;
  constexpr static double kDistalVelocity = kProximalVelocity*kMimicRatio;

  /// If (current dof - goalPosition) execution terminates. 
  constexpr static double kTolerance = 1e-3;

  ::dart::dynamics::ChainPtr mFinger;

  /// proximal, distal dofs
  ::dart::dynamics::DegreeOfFreedom* mProximalDof;
  ::dart::dynamics::DegreeOfFreedom* mDistalDof;

  /// Joint limits for proximal and distal dof.
  std::pair<double, double> mProximalLimits, mDistalLimits;

  ::dart::collision::CollisionDetectorPtr mCollisionDetector;
  ::dart::collision::CollisionOption mCollisionOptions;

  ::dart::collision::CollisionGroupPtr mProximalCollisionGroup;
  ::dart::collision::CollisionGroupPtr mDistalCollisionGroup;

  std::unique_ptr<std::promise<void>> mPromise;

  /// Control access to mPromise, mInExecution, mGoalPosition, mDistalOnly,
  /// mCollideWith 
  std::mutex mMutex;

  /// Flag for indicating execution of a command. 
  bool mInExecution;

  /// Desired end-position of proximal dof.
  double mProximalGoalPosition;

  /// Indicator that only distal finger is to be moved.
  bool mDistalOnly;

  ::dart::collision::CollisionGroupPtr mCollideWith;
   
  /// Helper method for step() to set variables for terminating an execution.
  void terminate();

};

using BarrettFingerPositionCommandExecutorPtr = std::shared_ptr<BarrettFingerPositionCommandExecutor>;


} // control
} // aikido

#endif
