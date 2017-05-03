#include <cassert>
#include <set>
#include <dart/common/StlHelpers.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/util/Spline.hpp>
#include "DynamicPath.h"
#include "ParabolicUtil.hpp"
#include "HauserParabolicSmootherHelpers.hpp"

namespace aikido {
namespace planner {
namespace parabolic {

std::unique_ptr<aikido::trajectory::Spline> doShortcut(
    const aikido::trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit,
    double _tolerance,
    aikido::util::RNG::result_type _rngSeed)
{
  auto stateSpace = _inputTrajectory.getStateSpace();

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = convertToDynamicPath(_inputTrajectory,
                                          _maxVelocity,
                                          _maxAcceleration);

  hauserParabolicSmoother::doShortcut(*dynamicPath.get(),
                               _feasibilityCheck,
                               _timelimit,
                               _tolerance,
                               _rngSeed);

  auto outputTrajectory =
            convertToSpline(*dynamicPath.get(), startTime, stateSpace);

  return outputTrajectory;
}

std::unique_ptr<trajectory::Spline> doBlend(
    const trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _blendRadius, int _blendIterations, double _tolerance)
{
  auto stateSpace = _inputTrajectory.getStateSpace();

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = convertToDynamicPath(_inputTrajectory,
                                          _maxVelocity,
                                          _maxAcceleration,
                                          false);

  hauserParabolicSmoother::doBlend(*dynamicPath.get(),
                                   _feasibilityCheck,
                                   _tolerance, _blendRadius,
                                   _blendIterations);

  auto outputTrajectory =
              convertToSpline(*dynamicPath.get(), startTime, stateSpace);

  return outputTrajectory;
}

std::unique_ptr<trajectory::Spline> doShortcutAndBlend(
    const trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit,
    double _blendRadius,
    int _blendIterations,
    double _tolerance,
    aikido::util::RNG::result_type _rngSeed)
{
  auto stateSpace = _inputTrajectory.getStateSpace();

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = convertToDynamicPath(_inputTrajectory,
                                          _maxVelocity,
                                          _maxAcceleration);

  hauserParabolicSmoother::doShortcut(*dynamicPath.get(),
                               _feasibilityCheck,
                               _timelimit,
                               _tolerance,
                               _rngSeed);

  hauserParabolicSmoother::doBlend(*dynamicPath.get(),
                                   _feasibilityCheck,
                                   _tolerance, _blendRadius,
                                   _blendIterations);

  auto outputTrajectory =
                convertToSpline(*dynamicPath.get(), startTime, stateSpace);

  return outputTrajectory;
}


} // namespace parabolic
} // namespace planner
} // namespace aikido
