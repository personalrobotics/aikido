#ifndef AIKIDO_CONTROL_BARRETTHANDPOSITIONCOMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETTHANDPOSITIONCOMANDEXECUTOR_HPP_
#include <chrono>
#include "BarrettFingerPositionCommandExecutor.hpp"
#include <dart/collision/CollisionDetector.h>
#include <dart/collision/Option.h>
#include <dart/collision/CollisionGroup.h>
#include <dart/collision/CollisionFilter.h>
#include <dart/dynamics/dynamics.h>
#include <Eigen/Dense>
#include <future>
#include <mutex>
#include <condition_variable>

namespace aikido {
namespace control {

/// Trajectory executor for BarretHand fingers. 
/// Assumes that fingers are underactuated: primal joint is actuated
/// and distal joint moves with certain mimic ratio (set to be Barret hand) 
/// until collision. 
class BarrettHandPositionCommandExecutor
{
public:
  /// Constructor.
  /// \param _fingers Fingers to be controlled by this Executor.
  ///        First 2 fingers should have 3 joints (spread, primal, distal),
  ///        each with 1 dof. 
  ///        Third finger should have 2 joints (primal, joint).
  /// \param _cyclePeriod Sets the cycle period of the execution thread.
  /// \param _collisionDetector CollisionDetector for detecting self collision
  ///        and collision with objects. 
  /// \param _collisionOptions Default is (enableContact=false, binaryCheck=true,
  ///        maxNumContacts = 1.) 
  ///        See dart/collison/Option.h for more information.
   BarrettHandPositionCommandExecutor(
    std::vector<::dart::dynamics::ChainPtr> _fingers,
    std::chrono::milliseconds _cyclePeriod,
    ::dart::collision::CollisionDetectorPtr _collisionDetector,
    ::dart::collision::Option _collisionOptions = ::dart::collision::Option(
      false, true, 1));

  virtual ~BarrettHandPositionCommandExecutor();
  
  /// Move the fingers until primal joints and spread are set to _positions.
  /// Distal joints moves with mimic ratio. 
  /// If setpoints/spread are above/below dof limit, this will map to dof limit.
  /// \param _positions End dof pose for primal joints and spread. 
  ///        First 3 should be for primal joints, the last element should be
  ///        for spread. If _positions are above/below joint limits,
  ///        the fingers will move only upto the limit.
  /// \param _duration Max duration to execute the trajectory. 
  ///        The trajectory may terminate early upon collision.
  /// \param _collideWith CollisionGroup to check collision with fingers.
  /// \return Future which becomes available when the execution completes.
  std::future<void> execute(
    Eigen::Matrix<double, 4, 1> _positions, 
    std::chrono::milliseconds _duration,
    ::dart::collision::CollisionGroupPtr _collideWith);

private:

  constexpr static int numFingers = 3;

  /// Number of dofs for each finger
  const int numDofs[3] = {3, 3, 2};
  
  std::vector<::dart::dynamics::ChainPtr> mFingers;

  /// Executor for primal and distal joints. 
  std::vector<BarrettFingerPositionCommandExecutor> mFingerCommandExecutors;

  ::dart::collision::CollisionDetectorPtr mCollisionDetector;
  ::dart::collision::Option mCollisionOptions;

  std::vector<::dart::collision::CollisionGroupPtr> mPrimalCollisionGroups;
  std::vector<::dart::collision::CollisionGroupPtr> mDistalCollisionGroups;

  std::chrono::milliseconds mCyclePeriod;

  std::unique_ptr<std::promise<void>> mPromise;

  /// Blocks spin() until execute(...) is called; paired with mSpinLock.
  std::condition_variable mCv;

  /// Lock for keeping spin thread alive. 
  /// Manages access on mPromise, mRunning, mSpread 
  /// mDuration, mCollideWith, mInExecution, mSetPoints
  std::mutex mSpinMutex;

  /// Thread for spin().
  std::thread mThread;
  
  /// Flag for killing spin thread. 
  bool mRunning;

  /// Flag for indicating execution of a command. 
  bool mInExecution;

  /// Values for executing a position command. 
  Eigen::Vector3d mSetPoints; 
  double mSpread;
  std::chrono::milliseconds mDuration;
  ::dart::collision::CollisionGroupPtr mCollideWith;


  /// To be executed on a separate thread.
  void spin(); 
};

} // control
} // aikido

#endif
