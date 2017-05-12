#include <cassert>
#include <set>
#include <dart/common/StlHelpers.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/util/Spline.hpp>
#include "DynamicPath.h"
#include "HauserParabolicSmootherHelpers.hpp"
#include "ParabolicUtil.hpp"

namespace aikido {
namespace planner {
namespace parabolic {

std::unique_ptr<aikido::trajectory::Spline> doShortcut(
    const aikido::trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    aikido::util::RNG& _rng,
    double _timelimit,
    double _checkResolution,
    double _tolerance)
{
  auto stateSpace = _inputTrajectory.getStateSpace();

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath
      = detail::convertToDynamicPath(_inputTrajectory, _maxVelocity, _maxAcceleration);

  detail::doShortcut(
      *dynamicPath,
      _feasibilityCheck,
      _timelimit,
      _checkResolution,
      _tolerance,
      _rng);

  auto outputTrajectory = detail::convertToSpline(*dynamicPath, startTime, stateSpace);

  return outputTrajectory;
}

std::unique_ptr<trajectory::Spline> doBlend(
    const trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _blendRadius,
    int _blendIterations,
    double _checkResolution,
    double _tolerance)
{
  auto stateSpace = _inputTrajectory.getStateSpace();

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = detail::convertToDynamicPath(
      _inputTrajectory, _maxVelocity, _maxAcceleration);

  detail::doBlend(
      *dynamicPath,
      _feasibilityCheck,
      _blendRadius,
      _blendIterations,
      _checkResolution,
      _tolerance);

  auto outputTrajectory = detail::convertToSpline(*dynamicPath, startTime, stateSpace);

  return outputTrajectory;
}

std::unique_ptr<trajectory::Spline> doShortcutAndBlend(
    const trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    aikido::util::RNG& _rng,
    double _timelimit,
    double _blendRadius,
    int _blendIterations,
    double _checkResolution,
    double _tolerance)
{
  auto stateSpace = _inputTrajectory.getStateSpace();

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath
      = detail::convertToDynamicPath(_inputTrajectory, _maxVelocity, _maxAcceleration);

  detail::doShortcut(
      *dynamicPath,
      _feasibilityCheck,
      _timelimit,
      _checkResolution,
      _tolerance,
      _rng);

  detail::doBlend(
      *dynamicPath,
      _feasibilityCheck,
      _blendRadius,
      _blendIterations,
      _checkResolution,
      _tolerance);

  auto outputTrajectory = detail::convertToSpline(*dynamicPath, startTime, stateSpace);

  return outputTrajectory;
}

} // namespace parabolic
} // namespace planner
} // namespace aikido
