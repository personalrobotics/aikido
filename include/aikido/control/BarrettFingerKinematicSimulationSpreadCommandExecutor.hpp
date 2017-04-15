#ifndef AIKIDO_CONTROL_BARRETFINGERKINEMATICSIMULATIONSPREADCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETFINGERKINEMATICSIMULATIONSPREADCOMMANDEXECUTOR_HPP_
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <future>
#include <mutex>
#include <condition_variable>
#include <chrono>

namespace aikido {
namespace control {

/// This executor mimics the spread behavior of BarretFinger.
/// It moves two finger spreads simultaneously to certain goal value;
/// it will stop prematurely if joint limit is reached or collision is detected.
class BarrettFingerKinematicSimulationSpreadCommandExecutor
{
public:
  /// Constructor.
  /// \param[in] fingers 2 fingers to be controlled by this Executor.
  /// \param[in] spread Index of spread dof
  /// \param[in] collisionDetector CollisionDetector to check collision with fingers.
  ///        If nullptr, default to FCLCollisionDetector.
  /// \param[in] collideWith CollisionGroup to check collision with fingers.
  ///        If nullptr, default to empty CollisionGroup
  /// \param[in] collisionOptions Default is (enableContact=false, binaryCheck=true,
  ///        maxNumContacts = 1.)
  BarrettFingerKinematicSimulationSpreadCommandExecutor(
    std::array<::dart::dynamics::ChainPtr, 2> fingers, size_t spread,
    ::dart::collision::CollisionDetectorPtr collisionDetector = nullptr,
    ::dart::collision::CollisionGroupPtr collideWith = nullptr,
    ::dart::collision::CollisionOption collisionOptions
      = ::dart::collision::CollisionOption(false, 1));

  /// Sets variables to move the spread joint by goalPosition,
  /// joint limit has reached, or until collision is detected.
  /// Must call step function after this for actual execution.
  /// \param goalPosition Desired angle of spread joint.
  /// \return future which becomes available when the execution completes.
  std::future<void> execute(double goalPosition);

  /// Moves the joint of the finger by fixed speed*timeSincePreviousCall
  /// until execute's goalPosition by spread dof or collision is detected.
  /// If multiple threads are accessing this function or the skeleton associated
  /// with this executor, it is necessary to lock the skeleton before
  /// calling this method.
  void step();

  /// Resets CollisionGroup to check collision.
  /// \param collideWith CollisionGroup to check collision with fingers.
  /// \return false if fails to change collideWith (during execution).
  bool setCollideWith(::dart::collision::CollisionGroupPtr collideWith);

private:
  constexpr static double kDofSpeed = 1;

  /// If (current dof - goalPosition) execution terminates.
  constexpr static double kTolerance = 1e-3;

  /// All fingers dof values should be the same (approx. within this tolerance).
  constexpr static double kDofTolerance = 1e-3;

  constexpr static int kNumFingers = 2;

  std::array<::dart::dynamics::ChainPtr, 2> mFingers;

  std::vector<::dart::dynamics::DegreeOfFreedom*> mSpreadDofs;

  std::pair<double, double> mDofLimits;

  ::dart::collision::CollisionDetectorPtr mCollisionDetector;
  ::dart::collision::CollisionGroupPtr mCollideWith;
  ::dart::collision::CollisionOption mCollisionOptions;

  ::dart::collision::CollisionGroupPtr mSpreadCollisionGroup;

  std::unique_ptr<std::promise<void>> mPromise;

  /// Controls access to mPromise, mInExecution, mGoalPosition, mCollideWith
  std::mutex mMutex;

  /// True if a command is in execution
  bool mInExecution;

  /// Time that step was last called.
  std::chrono::system_clock::time_point mTimeOfPreviousCall;

  /// Desired end spread value
  double mGoalPosition;
};

using BarrettFingerKinematicSimulationSpreadCommandExecutorPtr 
  = std::shared_ptr<BarrettFingerKinematicSimulationSpreadCommandExecutor>;

} // control
} // aikido

#endif
