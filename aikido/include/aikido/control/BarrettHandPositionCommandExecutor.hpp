#ifndef AIKIDO_CONTROL_BARRETTHANDPOSITIONCOMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETTHANDPOSITIONCOMANDEXECUTOR_HPP_
#include "BarrettFingerPositionCommandExecutor.hpp"
#include "BarrettFingerSpreadCommandExecutor.hpp"
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/Option.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <Eigen/Dense>
#include <future>
#include <mutex>
#include <chrono>

namespace aikido {
namespace control {

/// Position command executor for BarrettHand fingers. 
/// Assumes that fingers are underactuated: proximal joint is actuated
/// and distal joint moves with certain mimic ratio until collision.
/// If 
class BarrettHandPositionCommandExecutor
{
public:
  /// Constructor.
  /// \param _positionCommandExecutors 3 executors to control
  ///        proximal and distal joints of the fingers. The first two fingers
  ///        should have (spread, proximal, distal) joints. 
  ///        Third finger should have 2 joints (proximal, distal).
  /// \param _spreadCommandExecutor Executors to control
  ///        spreads of the fingers.
   BarrettHandPositionCommandExecutor(
    std::array<BarrettFingerPositionCommandExecutorPtr, 3> _positionCommandExecutors,
    BarrettFingerSpreadCommandExecutorPtr _spreadCommandExecutor);

  /// Set relevant variables for moving fingers.
  /// In order to move the fingers, step method should be called multiple times 
  /// until future returns.
  /// \param _goalPositions End dof pose for proximal joints and spread. 
  ///        First 3 should be for proximal joints, the last element should be
  ///        for spread. If _positions are above/below joint limits,
  ///        the fingers will move only upto the limit.
  /// \param _collideWith CollisionGroup to check collision with fingers.
  /// \return Future which becomes available when the execution completes.
  std::future<void> execute(
    Eigen::Matrix<double, 4, 1> _goalPositions, 
    ::dart::collision::CollisionGroupPtr _collideWith);

  /// Calls step method of finger executors.
  /// If multiple threads are accessing this function or skeleton associated
  /// with this executor, it is necessary to lock the skeleton before
  /// calling this method.
  /// \param _timeSincePreviousCall Time since previous call. 
  void step(double _timeSincePreviousCall);

private:

  constexpr static int kNumPositionExecutor = 3;
  constexpr static int kNumSpreadExecutor = 1; 
  constexpr static auto kWaitPeriod = std::chrono::milliseconds(1);

  /// Executor for proximal and distal joints. 
  std::array<BarrettFingerPositionCommandExecutorPtr, 3> mPositionCommandExecutors;
  BarrettFingerSpreadCommandExecutorPtr mSpreadCommandExecutor;

  std::unique_ptr<std::promise<void>> mPromise;

  std::vector<std::future<void>> mFingerFutures;

  /// Control access to mPromise, mInExecution
  /// mProximalGoalPositions, mSpreadGoalPositin, mSpread
  std::mutex mMutex;

  /// Flag for indicating execution of a command. 
  bool mInExecution;

  /// Values for executing a position and spread command. 
  Eigen::Vector3d mProximalGoalPositions; 
  double mSpreadGoalPosition;

  ::dart::collision::CollisionGroupPtr mCollideWith;

};

} // control
} // aikido

#endif
