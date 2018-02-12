#ifndef AIKIDO_CONTROL_BARRETTHANDKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETTHANDKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_

#include <future>
#include <mutex>
#include <Eigen/Dense>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/control/BarrettFingerKinematicSimulationPositionCommandExecutor.hpp"
#include "aikido/control/BarrettFingerKinematicSimulationSpreadCommandExecutor.hpp"
#include "aikido/control/PositionCommandExecutor.hpp"

namespace aikido {
namespace control {

AIKIDO_DECLARE_POINTERS(BarrettHandKinematicSimulationPositionCommandExecutor)

/// Position command executor for simulating BarrettHand.
///
/// See BarrettFingerKinematicSimulationPositionCommandExecutor and
/// BarrettFingerKinematicSimulationSpreadCommandExecutor for details.
class BarrettHandKinematicSimulationPositionCommandExecutor
    : public PositionCommandExecutor
{
public:
  /// Constructor.
  ///
  /// \param robot Robot to construct executor for
  /// \param prefix String (either "/right/" or "/left/") to specify hand
  /// \param collisionDetector CollisionDetector to check finger collisions
  ///        If nullptr, default to FCLCollisionDetector.
  /// \param collideWith CollisionGroup to check finger collisions
  ///        If nullptr, default to empty CollisionGroup.
  /// \param collisionOptions
  ///        Default is (enableContact=false, binaryCheck=true,
  ///        maxNumContacts = 1.) See dart/collison/Option.h for more
  ///        information
  BarrettHandKinematicSimulationPositionCommandExecutor(
      dart::dynamics::SkeletonPtr robot,
      const std::string& prefix,
      ::dart::collision::CollisionDetectorPtr collisionDetector = nullptr,
      ::dart::collision::CollisionGroupPtr collideWith = nullptr,
      ::dart::collision::CollisionOption collisionOptions
      = ::dart::collision::CollisionOption(false, 1));

  /// Move fingers to goalPositions. Call step() after this until future
  /// returns for actual execution.
  ///
  /// \param goalPositions Desired values for proximal and spread joints.
  ///        First 3 should specify proximal joints, last element should specify
  ///        spread. Joints will move only up to the joint limits.
  /// \return future which becomes available when the movement stops
  std::future<void> execute(const Eigen::VectorXd& goalPositions) override;

  /// \copydoc PositionCommandExecutor::step()
  /// \note Lock the Skeleton associated with this executor before calling this
  /// method.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  /// Sets CollisionGroup to check against for finger collisions.
  ///
  /// \param collideWith CollisionGroup to check finger collisions
  /// \return false if collideWith cannot be set (during execution)
  bool setCollideWith(::dart::collision::CollisionGroupPtr collideWith);

private:
  /// Set up mPositionCommandExecutors and mSpreadCommandExecutor.
  /// \param[in] robot Robot to construct hand executor for
  /// \param[in] prefix String (either "/right/" or "/left/") to specify hand
  void setupExecutors(
      dart::dynamics::SkeletonPtr robot, const std::string& prefix);

  /// Number of finger position executors
  constexpr static int kNumPositionExecutors = 3;

  /// Number of finger spread executors
  constexpr static int kNumSpreadExecutor = 1;

  /// Executor for proximal and distal joints
  std::array<BarrettFingerKinematicSimulationPositionCommandExecutorPtr,
             kNumPositionExecutors>
      mPositionCommandExecutors;

  /// Executor for spread joint
  BarrettFingerKinematicSimulationSpreadCommandExecutorPtr
      mSpreadCommandExecutor;

  /// Duration to wait for futures from executors
  constexpr static auto kWaitPeriod = std::chrono::milliseconds(0);

  /// Finger futures that are being waited on
  std::vector<std::future<void>> mFingerFutures;

  /// Collision detector to check finger collisions with
  ::dart::collision::CollisionDetectorPtr mCollisionDetector;

  /// Collision group to check for finger collisions against
  ::dart::collision::CollisionGroupPtr mCollideWith;

  /// Collision options to check finger collisions with
  ::dart::collision::CollisionOption mCollisionOptions;

  /// Whether a position command is being executed
  bool mInProgress;

  /// Promise whose future is returned by execute()
  std::unique_ptr<std::promise<void>> mPromise;

  /// Manages access to mCollideWith, mInProgress, mPromise
  std::mutex mMutex;
};

} // namespace control
} // namespace aikido

#endif
