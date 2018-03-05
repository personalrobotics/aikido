#ifndef AIKIDO_CONTROL_BARRETTFINGERKINEMATICSIMULATIONSPREADCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETTFINGERKINEMATICSIMULATIONSPREADCOMMANDEXECUTOR_HPP_

#include <future>
#include <mutex>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/common/pointers.hpp"
#include "aikido/control/PositionCommandExecutor.hpp"

namespace aikido {
namespace control {

AIKIDO_DECLARE_POINTERS(BarrettFingerKinematicSimulationSpreadCommandExecutor)

/// This executor mimics the spread behavior of a BarrettHand finger.
///
/// It spreads two fingers simultaneously to a desired goal value. It will stop
/// prematurely if a joint limit is reached or collision is detected.
class BarrettFingerKinematicSimulationSpreadCommandExecutor
    : public PositionCommandExecutor
{
public:
  /// Constructor.
  ///
  /// \param fingers 2 fingers to be controlled by this Executor.
  /// \param spread Index of spread dof
  /// \param collisionDetector CollisionDetector to check finger collisions
  ///        If nullptr, default to FCLCollisionDetector.
  /// \param collideWith CollisionGroup to check finger collisions
  ///        If nullptr, default to empty CollisionGroup.
  /// \param collisionOptions
  ///        Default is (enableContact=false, binaryCheck=true,
  ///        maxNumContacts = 1.) See dart/collison/Option.h for more
  ///        information
  BarrettFingerKinematicSimulationSpreadCommandExecutor(
      std::array<::dart::dynamics::ChainPtr, 2> fingers,
      std::size_t spread,
      ::dart::collision::CollisionDetectorPtr collisionDetector = nullptr,
      ::dart::collision::CollisionGroupPtr collideWith = nullptr,
      ::dart::collision::CollisionOption collisionOptions
      = ::dart::collision::CollisionOption(false, 1));

  /// Move the spread joint to goalPosition. Call step() after this until future
  /// returns for actual execution.
  ///
  /// Spread dof moves to goalPosition, joint limit, or until collision.
  ///
  /// \param goalPosition Desired angle for spread joint
  /// \return future which becomes available when movement stops
  std::future<void> execute(const Eigen::VectorXd& goalPosition) override;

  /// \copydoc BarrettHandKinematicSimulationPositionCommandExecutor::step()
  ///
  /// Moves the spread joint by dofVelocity * timeSincePreviousCall until either
  /// the spread dof reaches goalPosition, a joint limit is reached, or
  /// collision is detected.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  /* clang-format off */
  /// \copydoc BarrettHandKinematicSimulationPositionCommandExecutor::setCollideWith()
  /* clang-format on */
  bool setCollideWith(::dart::collision::CollisionGroupPtr collideWith);

private:
  /// Creates a CollisionGroup in the CollisionDetector for the fingers.
  void setFingerCollisionGroup();

  /// Spread joint velocity limit
  constexpr static double kDofSpeed = 1;

  /// Tolerance for whether goalPosition has been reached
  constexpr static double kTolerance = 1e-3;

  /// Tolerance for difference between finger dof values and limits
  constexpr static double kDofTolerance = 1e-3;

  /// Number of fingers to be spread
  constexpr static int kNumFingers = 2;

  /// Fingers to position command
  std::array<::dart::dynamics::ChainPtr, kNumFingers> mFingers;

  /// Spread DOFs
  std::vector<::dart::dynamics::DegreeOfFreedom*> mSpreadDofs;

  /// Joint limits for spread dof
  std::pair<double, double> mDofLimits;

  /// Collision detector to check finger collisions with
  ::dart::collision::CollisionDetectorPtr mCollisionDetector;

  /// Collision group to check for finger collisions against
  ::dart::collision::CollisionGroupPtr mCollideWith;

  /// Collision options to check finger collisions with
  ::dart::collision::CollisionOption mCollisionOptions;

  /// Collision group for spread links
  ::dart::collision::CollisionGroupPtr mSpreadCollisionGroup;

  /// Desired end spread value
  double mGoalPosition;

  /// Whether a position command is being executed
  bool mInProgress;

  /// Promise whose future is returned by execute()
  std::unique_ptr<std::promise<void>> mPromise;

  /// Manages access to mCollideWith, mGoalPosition, mInProgress, mPromise
  std::mutex mMutex;
};

} // namespace control
} // namespace aikido

#endif
