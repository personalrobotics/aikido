#ifndef AIKIDO_CONTROL_BARRETTFINGERKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETTFINGERKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_

#include <future>
#include <mutex>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/control/PositionCommandExecutor.hpp"

namespace aikido {
namespace control {

/// This executor mimics the behavior of a BarrettHand finger.
///
/// It moves a finger to a desired point; it may stop early if the joint limit
/// is reached or collision is detected. Only the proximal joint is actuated;
/// the distal joint moves with mimic ratio.
///
/// When collision is detected on the distal link, the finger stops.
/// When collision is detected on the proximal link, the distal link continues
/// to move until it reaches the joint limit or until distal collision is
/// detected.
class BarrettFingerKinematicSimulationPositionCommandExecutor
    : public PositionCommandExecutor
{
public:
  /// Constructor.
  ///
  /// \param finger Finger to be controlled by this Executor.
  /// \param proximal Index of proximal dof
  /// \param distal Index of distal dof
  /// \param timestep The time period that each call to step() should simulate
  /// \param collisionDetector CollisionDetector to check finger collisions
  ///        If nullptr, default to FCLCollisionDetector.
  /// \param collideWith CollisionGroup to check finger collisions
  ///        If nullptr, default to empty CollisionGroup.
  /// \param collisionOptions
  ///        Default is (enableContact=false, binaryCheck=true,
  ///        maxNumContacts = 1.) See dart/collison/Option.h for more
  ///        information
  BarrettFingerKinematicSimulationPositionCommandExecutor(
      ::dart::dynamics::ChainPtr finger,
      std::size_t proximal,
      std::size_t distal,
      std::chrono::milliseconds timestep,
      ::dart::collision::CollisionDetectorPtr collisionDetector = nullptr,
      ::dart::collision::CollisionGroupPtr collideWith = nullptr,
      ::dart::collision::CollisionOption collisionOptions
      = ::dart::collision::CollisionOption(false, 1));

  /// Open or close finger to goal position. Call step() after this until future
  /// returns for actual execution.
  ///
  /// Proximal dof moves to goalPosition, joint limit, or until collision.
  /// Distal dof follows with mimic ratio.
  ///
  /// \param goalPosition Desired angle for proximal joint
  /// \return future which becomes available when movement stops
  std::future<void> execute(const Eigen::VectorXd& goalPosition) override;

  /// Returns the mimic ratio, i.e. how much the distal joint moves relative to
  /// the proximal joint.
  /// \return mimic ratio
  constexpr static double getMimicRatio()
  {
    return kMimicRatio;
  }

  /// \copydoc BarrettHandKinematicSimulationPositionCommandExecutor::step()
  ///
  /// Moves the finger joint positions by dofVelocity * mTimestep until either
  /// the proximal dof reaches goalPosition, a joint limit is reached, or
  /// collision is detected.
  /// When collision is detected on the distal link, the finger stops.
  /// When collision is detected on the proximal link, the distal link continues
  /// to move until it either reaches mimicRatio * goalPosition, a joint limit
  /// is reached, or collision is detected.
  void step() override;

  /// \copydoc
  /// BarrettHandKinematicSimulationPositionCommandExecutor::setCollideWith()
  bool setCollideWith(::dart::collision::CollisionGroupPtr collideWith);

private:
  /// Creates a CollisionGroup in the CollisionDetector for the fingers.
  void setFingerCollisionGroup();

  /// Helper method for step() to set variables for terminating an execution.
  void terminate();

  /// How much the distal joint moves relative to the proximal joint. This ratio
  /// is only considered when both joints are moving.
  constexpr static double kMimicRatio = 0.333;

  /// Proximal joint velocity limit
  // TODO: read velocity limit from herb_description
  constexpr static double kProximalSpeed = 2.0;

  /// Distal joint velocity limit
  constexpr static double kDistalSpeed = kProximalSpeed * kMimicRatio;

  /// Finger to position command
  ::dart::dynamics::ChainPtr mFinger;

  /// Proximal DOF
  ::dart::dynamics::DegreeOfFreedom* mProximalDof;

  /// Distal DOF
  ::dart::dynamics::DegreeOfFreedom* mDistalDof;

  /// Joint limits for proximal and distal dof
  std::pair<double, double> mProximalLimits, mDistalLimits;

  /// Collision detector to check finger collisions with
  ::dart::collision::CollisionDetectorPtr mCollisionDetector;

  /// Collision group to check for finger collisions against
  ::dart::collision::CollisionGroupPtr mCollideWith;

  /// Collision options to check finger collisions with
  ::dart::collision::CollisionOption mCollisionOptions;

  /// Collision group for proximal link
  ::dart::collision::CollisionGroupPtr mProximalCollisionGroup;

  /// Collision group for distal link
  ::dart::collision::CollisionGroupPtr mDistalCollisionGroup;

  /// Desired end-position of proximal dof
  double mProximalGoalPosition;

  /// Desired end-position of distal dof
  double mDistalGoalPosition;

  /// Whether only the distal joint should move
  bool mDistalOnly;

  /// Whether a position command is being executed
  bool mInProgress;

  /// Promise whose future is returned by execute()
  std::unique_ptr<std::promise<void>> mPromise;

  /// Manages access to mCollideWith, mProximalGoalPosition,
  /// mDistalGoalPosition, mDistalOnly, mInProgress, mPromise
  std::mutex mMutex;
};

using BarrettFingerKinematicSimulationPositionCommandExecutorPtr
    = std::shared_ptr<BarrettFingerKinematicSimulationPositionCommandExecutor>;

} // namespace control
} // namespace aikido

#endif
