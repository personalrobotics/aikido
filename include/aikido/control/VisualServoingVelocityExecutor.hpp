#ifndef AIKIDO_CONTROL_VISUALSERVOINGVELOCITYEXECUTOR_HPP_
#define AIKIDO_CONTROL_VISUALSERVOINGVELOCITYEXECUTOR_HPP_

#include <future>
#include <mutex>

#include <dart/dynamics/Skeleton.hpp>

#include "aikido/control/Executor.hpp"
#include "aikido/control/JacobianExecutor.hpp"
#include "aikido/control/util.hpp"

namespace aikido {
namespace control {

/// Executes end-effector R3 linear velocity
/// command towards a perceived target
/// Uses underlying JacobianVelocityExecutor
/// on its own thread.
///
class VisualServoingVelocityExecutor : public Executor
{
public:
  /// Paramters for Vector Field Planner
  struct Properties
  {
    static constexpr double DEFAULT_GOAL_TOLERANCE = 0.01; // m
    static constexpr double DEFAULT_APPROACH_VEL = 0.05;   // m/s
    static constexpr double DEFAULT_TIMEOUT = 0.5;         // s
    static constexpr long DEFAULT_THREAD_PERIOD = 10L;     // ms
    /// Struct Constructor
    /// \param[in] goalTolerance (m) How close we can be to the goal
    ///            before terminating the action.
    /// \param[in] approachVelocity (m/s) How fast to move towards goal
    /// \param[in] timeout (s) how long to go with failed perception
    ///            before killing command (0 = never kill command)
    /// \param[in] threadPeriod (ms) thread cycle period (if running
    ///            on separate thread)
    Properties(
        double goalTolerance = DEFAULT_GOAL_TOLERANCE,
        double approachVelocity = DEFAULT_APPROACH_VEL,
        std::chrono::duration<double> timeout
        = std::chrono::duration<double>(DEFAULT_TIMEOUT),
        std::chrono::milliseconds threadPeriod
        = std::chrono::milliseconds(DEFAULT_THREAD_PERIOD))
      : mGoalTolerance(goalTolerance)
      , mApproachVelocity(approachVelocity)
      , mTimeout(timeout)
      , mThreadPeriod(threadPeriod){
            // Do nothing
        };

    double mGoalTolerance;
    double mApproachVelocity;
    std::chrono::duration<double> mTimeout;
    std::chrono::milliseconds mThreadPeriod;
  };

  /// Constructor.
  ///
  /// \param eeNode End effector body node (current position =
  /// getWorldTransform()) \param executor Base jacobian velocity executor.
  ///                Default: kinematic, constructed from BodyNode
  /// \param properties See ::Properties for more info.
  explicit VisualServoingVelocityExecutor(
      ::dart::dynamics::BodyNode* eeNode,
      std::shared_ptr<JacobianVelocityExecutor> executor = nullptr,
      Properties properties = Properties());

  virtual ~VisualServoingVelocityExecutor();

  /// Cancel current command and update properties
  void resetProperties(Properties properties = Properties());

  /// Get current properties
  Properties getProperties() const;

  /// Execute a Visual Servoing Command, setting future upon completion
  /// \note Future should return 0 on success or timeout.
  ///
  /// \param perception goal position in DART World Frame
  virtual std::future<int> execute(
      std::function<std::shared_ptr<Eigen::Isometry3d>(void)> perception)
  {
    return this->execute(perception, std::chrono::system_clock::now());
  }
  virtual std::future<int> execute(
      std::function<std::shared_ptr<Eigen::Isometry3d>(void)> perception,
      const std::chrono::system_clock::time_point& timepoint);

  /// \copydoc Executor::step()
  ///
  /// If multiple threads are accessing this function or the skeleton associated
  /// with this executor, it is necessary to lock the skeleton before
  /// calling this method.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  /// Cancels the current command.
  void cancel();

private:
  /// End Effector Body Node Name
  ::dart::dynamics::BodyNode* mEENode;

  /// Underlying JacobianVelocityExecutor
  std::shared_ptr<JacobianVelocityExecutor> mExecutor;

  /// Perception Function (if empty, then executor not in progress)
  std::function<std::shared_ptr<Eigen::Isometry3d>(void)> mPerception;

  /// Promise whose future is returned by execute()
  std::unique_ptr<std::promise<int>> mPromise;

  /// Manages access to mPerception, mPromise
  mutable std::mutex mMutex;

  /// Properties (see documentation above)
  Properties mProperties;

  /// Future for underlying executor call
  std::future<int> mFuture;

  /// Time of command start
  std::chrono::system_clock::time_point mLastPerceivedTime;
};

} // namespace control
} // namespace aikido

#endif
