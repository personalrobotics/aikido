#ifndef AIKIDO_CONTROL_POSITIONCOMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_POSITIONCOMANDEXECUTOR_HPP_
#include <Eigen/Dense>
#include <future>

namespace aikido {
namespace control {

/// Abstract class for executing position commands.
class PositionCommandExecutor
{
public:
  virtual ~PositionCommandExecutor() = default;

  /// Execute hand to goalPosition
  /// \param goalPositions Goal positions
  /// \return future which becomes available when the execution completes.
  virtual std::future<void> execute(const Eigen::VectorXd& goalPositions) = 0;

  // Step once.
  virtual void step() = 0;
};

using PositionCommandExecutorPtr = std::shared_ptr<PositionCommandExecutor>;


} // control
} // aikido

#endif
