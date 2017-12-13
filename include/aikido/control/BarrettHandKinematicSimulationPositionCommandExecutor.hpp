#ifndef AIKIDO_CONTROL_BARRETTHANDKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETTHANDKINEMATICSIMULATIONPOSITIONCOMMANDEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <mutex>
#include <Eigen/Dense>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <aikido/control/BarrettFingerKinematicSimulationPositionCommandExecutor.hpp>
#include <aikido/control/BarrettFingerKinematicSimulationSpreadCommandExecutor.hpp>
#include <aikido/control/PositionCommandExecutor.hpp>

using Vector1d = Eigen::Matrix<double, 1, 1>;

namespace aikido {
namespace control {

/// Position command executor for simulating BarrettHand fingers.
/// Assumes that fingers are underactuated: proximal joint is actuated
/// and distal joint moves with certain mimic ratio until collision.
class BarrettHandKinematicSimulationPositionCommandExecutor
    : public PositionCommandExecutor
{
public:
  /// Constructor.
  /// \param[in] robot Robot to construct executor for
  /// \param[in] prefix String (either "/right/" or "/left/") to specify hand
  /// \param[in] collisionDetector CollisionDetector to check collision with
  /// fingers.
  ///        If nullptr, default to FCLCollisionDetector.
  /// \param[in] collideWith CollisionGroup to check collision with fingers.
  ///        If nullptr, default to empty CollisionGroup
  BarrettHandKinematicSimulationPositionCommandExecutor(
      dart::dynamics::SkeletonPtr robot,
      const std::string& prefix,
      ::dart::collision::CollisionDetectorPtr collisionDetector = nullptr,
      ::dart::collision::CollisionGroupPtr collideWith = nullptr);

  /// Move fingers to a goal configuration. In order to actually move the
  /// fingers, step method should be called multiple times until future returns.
  /// \param _goalPositions End dof pose for proximal joints and spread.
  ///        First 3 should be for proximal joints, the last element should be
  ///        for spread. If _positions are above/below joint limits,
  ///        the fingers will move only up to the limit.
  /// \return Future which becomes available when the execution completes.
  std::future<void> execute(const Eigen::VectorXd& goalPositions) override;

  /// \copydoc PositionCommandExecutor::step()
  ///
  /// If multiple threads are accessing this function or the skeleton associated
  /// with this executor, it is necessary to lock the skeleton before
  /// calling this method.
  void step() override;

  /// Move fingers to a goal configuration.
  /// \param _goalPositions End dof pose for proximal joints and spread.
  ///        First 3 should be for proximal joints, the last element should be
  ///        for spread. If _positions are above/below joint limits,
  ///        the fingers will move only up to the limit.
  /// \param dt Step time
  void jump(const Eigen::VectorXd& goalPositions, double dt = 0.01);

  /// Resets CollisionGroup to check collision with fingers.
  /// \param _collideWith CollisionGroup to check collision with fingers.
  /// \return false if fails to change collideWith (during execution).
  bool setCollideWith(::dart::collision::CollisionGroupPtr collideWith);

private:
  /// Set up mPositionCommandExecutors and mSpreadCommandExecutor.
  /// \param[in] robot Robot to construct hand executor for
  /// \param[in] prefix String (either "/right/" or "/left/") to specify hand
  void setupExecutors(
      dart::dynamics::SkeletonPtr robot, const std::string& prefix);

  constexpr static int kNumPositionExecutor = 3;
  constexpr static int kNumSpreadExecutor = 1;
  constexpr static auto kWaitPeriod = std::chrono::milliseconds(1);

  /// Executor for proximal and distal joints.
  std::array<BarrettFingerKinematicSimulationPositionCommandExecutorPtr,
             kNumPositionExecutor>
      mPositionCommandExecutors;
  BarrettFingerKinematicSimulationSpreadCommandExecutorPtr
      mSpreadCommandExecutor;

  std::unique_ptr<std::promise<void>> mPromise;

  std::vector<std::future<void>> mFingerFutures;

  /// Control access to mPromise, mInExecution
  /// mProximalGoalPositions, mSpreadGoalPositin, mSpread
  std::mutex mMutex;

  /// Flag for indicating execution of a command.
  bool mInExecution;

  /// Values for executing a position and spread command.
  Eigen::Vector3d mProximalGoalPositions;
  Vector1d mSpreadGoalPosition;

  ::dart::collision::CollisionDetectorPtr mCollisionDetector;
  ::dart::collision::CollisionGroupPtr mCollideWith;
};

using BarrettHandKinematicSimulationPositionCommandExecutorPtr
    = std::shared_ptr<BarrettHandKinematicSimulationPositionCommandExecutor>;

} // namespace control
} // namespace aikido

#endif
