#ifndef AIKIDO_CONTROL_BARRETFINGERKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETFINGERKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_
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

/// This executor mimics the behavior of BarretFinger.
/// It moves a finger to a desired point; it may stop early if
/// joint limit is reached or collision is detected.
/// Only the proximal joint is actuated; the distal joint moves with mimic ratio.
/// When collision is detected on the distal link, the finger stops.
/// When collision is detected on the proximal link, the distal link moves
/// until it reaches joint limit or until distal collision is detected.
class BarrettFingerKinematicSimulationPositionCommandExecutor
{
public:
  /// Constructor.
  /// \param[in] finger Finger to be controlled by this Executor.
  /// \param[in] proximal Index of proximal dof
  /// \param[in] distal Index of distal dof
  /// \param[in] collisionDetector CollisionDetector to check collision with fingers.
  ///        If nullptr, default to FCLCollisionDetector.
  /// \param[in] collideWith CollisionGroup to check collision with fingers.
  ///        If nullptr, default to empty CollisionGroup.
  /// \param[in] collisionOptions Default is (enableContact=false, binaryCheck=true,
  ///        maxNumContacts = 1.)
  ///        See dart/collison/Option.h for more information
  BarrettFingerKinematicSimulationPositionCommandExecutor(
    ::dart::dynamics::ChainPtr finger, size_t proximal, size_t distal,
    ::dart::collision::CollisionDetectorPtr collisionDetector = nullptr,
    ::dart::collision::CollisionGroupPtr collideWith = nullptr,
    ::dart::collision::CollisionOption collisionOptions
      = ::dart::collision::CollisionOption(false, 1));

  /// Sets variables to move the finger joints.
  /// proximal dof moves to goalPosition, joint limit, or until collision.
  /// Step method should be called multiple times until future returns.
  /// Distal dof follows with mimic ratio.
  /// \param goalPosition Desired angle of proximal joint.
  /// \return future Becomes available when the execution completes.
  std::future<void> execute(double goalPosition);

  /// Returns mimic ratio, i.e. how much the distal joint moves relative to
  /// the proximal joint. The joint movements follow
  /// this ratio only when both joints are moving.
  /// \return mimic ratio.
  constexpr static double getMimicRatio() { return kMimicRatio; }

  /// Moves the joints of the finger by dofVelocity*timeSincePreviousCall
  /// until execute's goalPosition by primary dof or collision is detected.
  /// If proximal link is in collision, distal link moves until
  /// mimicRatio*goalPosition. If distal link is in collision, execution stops.
  /// If multiple threads are accessing this function or skeleton associated
  /// with this executor, it is necessary to lock the skeleton before
  /// calling this method.
  /// \param[in] timeSincePreviousCall Time interval to take.
  void step(const std::chrono::milliseconds& timeSincePreviousCall);

  /// Resets CollisionGroup to check collision with fingers.
  /// \param _collideWith CollisionGroup to check collision with fingers.
  /// \return false if fails to change collideWith (during execution).
  bool setCollideWith(::dart::collision::CollisionGroupPtr collideWith);

private:
  constexpr static double kMimicRatio = 0.333;
  constexpr static double kProximalSpeed = 1;
  constexpr static double kDistalSpeed = kProximalSpeed*kMimicRatio;

  /// If (current dof - goalPosition) execution terminates.
  constexpr static double kTolerance = 1e-3;

  ::dart::dynamics::ChainPtr mFinger;

  /// proximal, distal dofs
  ::dart::dynamics::DegreeOfFreedom* mProximalDof;
  ::dart::dynamics::DegreeOfFreedom* mDistalDof;

  /// Joint limits for proximal and distal dof.
  std::pair<double, double> mProximalLimits, mDistalLimits;

  ::dart::collision::CollisionDetectorPtr mCollisionDetector;
  ::dart::collision::CollisionGroupPtr mCollideWith;
  ::dart::collision::CollisionOption mCollisionOptions;

  ::dart::collision::CollisionGroupPtr mProximalCollisionGroup;
  ::dart::collision::CollisionGroupPtr mDistalCollisionGroup;

  std::unique_ptr<std::promise<void>> mPromise;

  /// Flag for indicating execution of a command.
  bool mInExecution;

  /// Control access to mPromise, mInExecution, mGoalPosition, mDistalOnly,
  /// mCollideWith
  std::mutex mMutex;

  /// Desired end-position of proximal dof.
  double mProximalGoalPosition;

  /// Indicator that only distal finger is to be moved.
  bool mDistalOnly;

  /// Helper method for step() to set variables for terminating an execution.
  void terminate();

};

using BarrettFingerKinematicSimulationPositionCommandExecutorPtr
  = std::shared_ptr<BarrettFingerKinematicSimulationPositionCommandExecutor>;


} // control
} // aikido

#endif
