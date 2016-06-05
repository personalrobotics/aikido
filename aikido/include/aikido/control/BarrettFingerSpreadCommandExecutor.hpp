#ifndef AIKIDO_CONTROL_BARRETFINGERSPREADCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETFINGERSPREADCOMMANDEXECUTOR_HPP_
#include <dart/collision/CollisionDetector.h>
#include <dart/collision/Option.h>
#include <dart/collision/CollisionGroup.h>
#include <dart/collision/CollisionFilter.h>
#include <dart/dynamics/dynamics.h>
#include <future>
#include <mutex>
#include <condition_variable>

namespace aikido {
namespace control {

/// This executor mimics the spread behavior of BarretFinger. 
/// It moves a finger's spread to its goal value, joint limit, or until collision.
class BarrettFingerSpreadCommandExecutor
{
public:
  /// Constructor.
  /// \param _finger finger to be controlled by this Executor.
  /// \param _spread Index of spread dof
  /// \param _collisionDetector CollisionDetector for detecting self collision
  ///        and collision with objects. 
  /// \param _collisionOptions Default is (enableContact=false, binaryCheck=true,
  ///        maxNumContacts = 1.) 
  ///        See dart/collison/Option.h for more information 
  BarrettFingerSpreadCommandExecutor(
    ::dart::dynamics::ChainPtr _finger, int _spread, 
    ::dart::collision::CollisionDetectorPtr _collisionDetector,
    ::dart::collision::Option _collisionOptions = ::dart::collision::Option(
      false, true, 1));

  /// Sets varialbes to move the spread joint by _goalPosition,
  /// joint limit has reached, or until collision is detected.
  /// Must call step function after this for actual execution. 
  /// \param _goalPosition Desired angle of spread joint.
  /// \param _collideWith CollisionGroup to check collision with fingers.
  /// \return future which becomes available when the execution completes.
  std::future<void> execute(double _goalPosition,
    ::dart::collision::CollisionGroupPtr _collideWith);

  /// Moves the joint of the finger by certain velocity*_timeSIncePreviousCall
  /// until execute's goalPosition by spread dof or collision is detected.
  /// If multiple threads are accessing this function or skeleton associated
  /// with this executor, it is necessary to lock the skeleton before
  /// calling this method.
  /// \param _timeSincePreviousCall Time since previous call. 
  void step(double _timeSincePreviousCall);   


private:
  constexpr static double kDofVelocity = 0.01;

  /// If (current dof - goalPosition) execution terminates. 
  constexpr static double kTolerance = 1e-3;

  ::dart::dynamics::ChainPtr mFinger; 

  ::dart::dynamics::DegreeOfFreedom* mSpreadDof;

  std::pair<double, double> mDofLimits;

  ::dart::collision::CollisionDetectorPtr mCollisionDetector;
  ::dart::collision::Option mCollisionOptions;

  ::dart::collision::CollisionGroupPtr mSpreadCollisionGroup;

  std::unique_ptr<std::promise<void>> mPromise;

  /// Controls access to mPromise, mInExecution, mGoalPosition, mCollideWith
  std::mutex mMutex;

  /// True if a command is in execution
  bool mInExecution;

  /// Desired end spread value
  double mGoalPosition;

  ::dart::collision::CollisionGroupPtr mCollideWith;
};

using BarrettFingerSpreadCommandExecutorPtr = std::shared_ptr<BarrettFingerSpreadCommandExecutor>;

} // control
} // aikido

#endif
