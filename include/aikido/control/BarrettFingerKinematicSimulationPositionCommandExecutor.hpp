#ifndef AIKIDO_CONTROL_BARRETFINGERKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETFINGERKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_

#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/common/pointers.hpp"
#include <aikido/control/PositionCommandExecutor.hpp>

namespace aikido {
namespace control {

AIKIDO_DECLARE_POINTERS(BarrettFingerKinematicSimulationPositionCommandExecutor)

/// This executor mimics the behavior of BarretFinger.
/// It moves a finger to a desired point; it may stop early if
/// joint limit is reached or collision is detected.
/// Only the proximal joint is actuated; the distal joint moves with mimic
/// ratio.
/// When collision is detected on the distal link, the finger stops.
/// When collision is detected on the proximal link, the distal link moves
/// until it reaches joint limit or until distal collision is detected.
class BarrettFingerKinematicSimulationPositionCommandExecutor
    : public PositionCommandExecutor
{
public:
  /// Constructor.
  /// \param[in] finger Finger to be controlled by this Executor.
  /// \param[in] proximal Index of proximal dof
  /// \param[in] distal Index of distal dof
  /// \param[in] collisionDetector CollisionDetector to check collision with
  /// fingers.
  ///        If nullptr, default to FCLCollisionDetector.
  /// \param[in] collideWith CollisionGroup to check collision with fingers.
  ///        If nullptr, default to empty CollisionGroup.
  /// \param[in] collisionOptions Default is (enableContact=false,
  /// binaryCheck=true,
  ///        maxNumContacts = 1.)
  ///        See dart/collison/Option.h for more information
  BarrettFingerKinematicSimulationPositionCommandExecutor(
      ::dart::dynamics::ChainPtr finger,
      std::size_t proximal,
      std::size_t distal,
      ::dart::collision::CollisionDetectorPtr collisionDetector = nullptr,
      ::dart::collision::CollisionGroupPtr collideWith = nullptr,
      ::dart::collision::CollisionOption collisionOptions
      = ::dart::collision::CollisionOption(false, 1));

  /// Open/close fingers to goal configuration.
  /// Proximal dof moves to goalPosition, joint limit, or until collision.
  /// Distal dof follows with mimic ratio.
  /// Call step after this for actual execution until future returns.
  /// \param goalPosition Desired angle of proximal joint.
  /// \return future Becomes available when the execution completes.
  std::future<void> execute(const Eigen::VectorXd& goalPosition) override;

  /// Returns mimic ratio, i.e. how much the distal joint moves relative to
  /// the proximal joint. The joint movements follow
  /// this ratio only when both joints are moving.
  /// \return mimic ratio.
  constexpr static double getMimicRatio()
  {
    return kMimicRatio;
  }

  /// Moves the joints of the finger by dofVelocity*timeSincePreviousCall
  /// until execute's goalPosition by primary dof or collision is detected.
  /// If proximal link is in collision, distal link moves until
  /// mimicRatio*goalPosition. If distal link is in collision, execution stops.
  /// If multiple threads are accessing this function or skeleton associated
  /// with this executor, it is necessary to lock the skeleton before
  /// calling this method.
  void step() override;

  /// Resets CollisionGroup to check collision with fingers.
  /// \param _collideWith CollisionGroup to check collision with fingers.
  /// \return false if fails to change collideWith (during execution).
  bool setCollideWith(::dart::collision::CollisionGroupPtr collideWith);

private:
  /// Creates a CollisionGroup in the CollisionDetector for the fingers.
  void setFingerCollisionGroup();

  constexpr static double kMimicRatio = 0.333;
  // TODO: read velocity limit from herb_description
  constexpr static double kProximalSpeed = 2.0;
  constexpr static double kDistalSpeed = kProximalSpeed * kMimicRatio;

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

  /// Time that step was last called.
  std::chrono::system_clock::time_point mTimeOfPreviousCall;

  /// Control access to mPromise, mInExecution, mGoalPosition, mDistalOnly,
  /// mCollideWith
  std::mutex mMutex;

  /// Desired end-position of proximal dof.
  double mProximalGoalPosition;

  /// Desired end-position of distal dof.
  double mDistalGoalPosition;

  /// Indicator that only distal finger is to be moved.
  bool mDistalOnly;

  /// Helper method for step() to set variables for terminating an execution.
  void terminate();
};

} // namespace control
} // namespace aikido

#endif
