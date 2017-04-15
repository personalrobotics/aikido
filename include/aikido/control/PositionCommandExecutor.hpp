#ifndef AIKIDO_CONTROL_POSITIONCOMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_POSITIONCOMANDEXECUTOR_HPP_
#include <Eigen/Dense>
#include <future>

namespace aikido {
namespace control {

/// Abstract class for executing position commands. A position command requests
/// several degrees of freedom to move to a specified configuration and provides
/// feedback when the move is complete.
class PositionCommandExecutor
{
public:
  virtual ~PositionCommandExecutor() = default;

  /// Execute hand to goalPosition
  /// \param goalPositions Goal positions
  /// \return future which becomes available when movement stops (which may not
  ///         be when the hand reaches the specified goalPositions due to e.g.
  ///         collision)
  virtual std::future<void> execute(const Eigen::VectorXd& goalPositions) = 0;

  // Step once.
  virtual void step() = 0;
};

using PositionCommandExecutorPtr = std::shared_ptr<PositionCommandExecutor>;


} // control
} // aikido

#endif
