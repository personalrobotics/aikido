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
  /// Constructor.
  ///
  /// \param timestep The time period that each call to step() should simulate
  PositionCommandExecutor(std::chrono::milliseconds timestep);

  virtual ~PositionCommandExecutor() = default;

  /// Move hand to goalPosition.
  ///
  /// \param goalPositions Goal positions
  /// \return future which becomes available when movement stops
  virtual std::future<void> execute(const Eigen::VectorXd& goalPositions) = 0;

  // Step once.
  virtual void step() = 0;

  /// Get the current timestep.
  virtual std::chrono::milliseconds getTimestep() const;

  /// Set the current timestep.
  virtual void setTimestep(std::chrono::milliseconds timestep);

protected:
  /// Time period that each call to step() should simulate
  std::chrono::milliseconds mTimestep;
};

using PositionCommandExecutorPtr = std::shared_ptr<PositionCommandExecutor>;

} // namespace control
} // namespace aikido

#endif
