#ifndef AIKIDO_CONTROL_POSITIONCOMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_POSITIONCOMANDEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <Eigen/Dense>

namespace aikido {
namespace control {

/// Abstract class for executing position commands.
///
/// A position command requests several degrees of freedom to move to a
/// specified configuration and provides feedback when the move is complete.
///
/// \note A move may complete before the DOFs reach the specified configuration
/// due to e.g. collision.
class PositionCommandExecutor
{
public:
  virtual ~PositionCommandExecutor() = default;

  /// Move hand to goalPosition.
  ///
  /// \param goalPositions Goal positions
  /// \return future which becomes available when movement stops
  virtual std::future<void> execute(const Eigen::VectorXd& goalPositions) = 0;

  /// Step to a point in time.
  /// \note \c timepoint can be a time in the future to enable faster than
  /// real-time execution.
  ///
  /// \param timepoint Time to simulate to
  virtual void step(const std::chrono::system_clock::time_point& timepoint) = 0;

protected:
  /// Time of previous call
  std::chrono::system_clock::time_point mTimeOfPreviousCall;
};

using PositionCommandExecutorPtr = std::shared_ptr<PositionCommandExecutor>;

} // namespace control
} // namespace aikido

#endif
