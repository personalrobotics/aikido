#ifndef AIKIDO_CONTROL_BARRETTHANDPOSITIONCOMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETTHANDPOSITIONCOMANDEXECUTOR_HPP_
#include <Eigen/Dense>
#include <future>

namespace aikido {
namespace control {

/// Abstract class for position commands for a Barrett Hand.
class BarrettHandPositionCommandExecutor
{
public:
  virtual ~BarrettHandPositionCommandExecutor() = default;

  /// Execute hand to goalPosition
  /// \param goalPositions End dof pose for proximal joints and spread.
  /// \return Future which becomes available when the execution completes.
  virtual std::future<void> execute(
    Eigen::Matrix<double, 4, 1> goalPositions) = 0;

  virtual void step(double timeSincePreviousCall) = 0;
};

using BarrettHandPositionCommandExecutorPtr = std::shared_ptr<BarrettHandPositionCommandExecutor>;


} // control
} // aikido

#endif
