#ifndef AIKIDO_CONTROL_BARRETFINGERSPREADCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETFINGERSPREADCOMMANDEXECUTOR_HPP_
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/Option.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <future>
#include <mutex>
#include <condition_variable>

namespace aikido {
namespace control {

/// This executor mimics the spread behavior of BarretFinger. 
/// It moves two finger spreads simultaneously to certain goal value;
/// it will stop prematurely if joint limit is reached or collision is detected.
class BarrettFingerSpreadCommandExecutor
{
public:
  /// Constructor.
  /// \param _fingers 2 fingers to be controlled by this Executor.
  /// \param _spread Index of spread dof
  /// \param _collisionDetector CollisionDetector for detecting self collision
  ///        and collision with objects. 
  /// \param _collisionOptions Default is (enableContact=false, binaryCheck=true,
  ///        maxNumContacts = 1.) 
  ///        See dart/collison/Option.h for more information 
  BarrettFingerSpreadCommandExecutor(
    std::array<::dart::dynamics::ChainPtr, 2> _fingers, int _spread, 
    ::dart::collision::CollisionDetectorPtr _collisionDetector,
    ::dart::collision::CollisionOption _collisionOptions
      = ::dart::collision::CollisionOption(false, 1));

  /// Sets variables to move the spread joint by _goalPosition,
  /// joint limit has reached, or until collision is detected.
  /// Must call step function after this for actual execution. 
  /// \param _goalPosition Desired angle of spread joint.
  /// \param _collideWith CollisionGroup to check collision with fingers.
  /// \return future which becomes available when the execution completes.
  std::future<void> execute(double _goalPosition,
    ::dart::collision::CollisionGroupPtr _collideWith);

  /// Moves the joint of the finger by certain velocity*_timeSincePreviousCall
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

  /// All fingers dof values should be the same (approx. within this tolerance).
  constexpr static double kDofTolerance = 1e-3;

  constexpr static int kNumFingers = 2;

  std::array<::dart::dynamics::ChainPtr, 2> mFingers; 

  std::vector<::dart::dynamics::DegreeOfFreedom*> mSpreadDofs;

  std::pair<double, double> mDofLimits;

  ::dart::collision::CollisionDetectorPtr mCollisionDetector;
  ::dart::collision::CollisionOption mCollisionOptions;

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
