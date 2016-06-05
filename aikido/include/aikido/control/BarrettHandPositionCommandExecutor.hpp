#ifndef AIKIDO_CONTROL_BARRETTHANDPOSITIONCOMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETTHANDPOSITIONCOMANDEXECUTOR_HPP_
#include "BarrettFingerPositionCommandExecutor.hpp"
#include "BarrettFingerSpreadCommandExecutor.hpp"
#include <dart/collision/CollisionDetector.h>
#include <dart/collision/Option.h>
#include <dart/collision/CollisionGroup.h>
#include <dart/collision/CollisionFilter.h>
#include <dart/dynamics/dynamics.h>
#include <Eigen/Dense>
#include <future>
#include <mutex>
#include <chrono>

namespace aikido {
namespace control {

/// Position command executor for BarrettHand fingers. 
/// Assumes that fingers are underactuated: primal joint is actuated
/// and distal joint moves with certain mimic ratio until collision.
/// If 
class BarrettHandPositionCommandExecutor
{
public:
  /// Constructor.
  /// \param _positionCommandExecutors 3 executors to control
  ///        primal and distal joints of the fingers. 
  /// \param _spreadCommandExecutors 2 executors to control
  ///        spreads of the fingers.
  ///        Third finger should have 2 joints (primal, joint).
   BarrettHandPositionCommandExecutor(
    std::vector<BarrettFingerPositionCommandExecutorPtr> _positionCommandExecutors,
    std::vector<BarrettFingerSpreadCommandExecutorPtr> _spreadCommandExecutors);

  /// Set relevant variables for moving fingers.
  /// In order to move the fingers, step method should be called multiple times 
  /// until future returns.
  /// \param _goalPositions End dof pose for primal joints and spread. 
  ///        First 3 should be for primal joints, the last element should be
  ///        for spread. If _positions are above/below joint limits,
  ///        the fingers will move only upto the limit.
  /// \param _collideWith CollisionGroup to check collision with fingers.
  /// \return Future which becomes available when the execution completes.
  std::future<void> execute(
    Eigen::Matrix<double, 4, 1> _goalPositions, 
    ::dart::collision::CollisionGroupPtr _collideWith);

  /// Calls step method of finger executors.
  /// \param _timeSincePreviousCall Time since previous call. 
  void step(double _timeSincePreviousCall);

private:

  constexpr static int kNumFingers = 3; 
  constexpr static int kNumSpreadFingers = 2;
  constexpr static auto kWaitPeriod = std::chrono::milliseconds(1);

  /// Executor for primal and distal joints. 
  std::vector<BarrettFingerPositionCommandExecutorPtr> mPositionCommandExecutors;
  std::vector<BarrettFingerSpreadCommandExecutorPtr> mSpreadCommandExecutors;

  std::unique_ptr<std::promise<void>> mPromise;

  std::vector<std::future<void>> mFingerFutures;

  /// Control access to mPromise, mInExecution
  /// mPrimalGoalPositions, mSpreadGoalPositin, mSpread
  std::mutex mMutex;

  /// Flag for indicating execution of a command. 
  bool mInExecution;

  /// Values for executing a position and spread command. 
  Eigen::Vector3d mPrimalGoalPositions; 
  double mSpreadGoalPosition;

  ::dart::collision::CollisionGroupPtr mCollideWith;

};

} // control
} // aikido

#endif
