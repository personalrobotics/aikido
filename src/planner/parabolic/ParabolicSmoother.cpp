#include <cassert>
#include <set>
#include <dart/common/StlHelpers.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/util/Spline.hpp>
#include "DynamicPath.h"
#include "ParabolicUtil.hpp"
#include "HauserParabolicSmoother.hpp"

namespace aikido {
namespace planner {
namespace parabolic {

std::unique_ptr<aikido::trajectory::Spline> doShortcut(
    const aikido::trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit,
    aikido::util::RNG::result_type _rngSeed)
{
  auto stateSpace = _inputTrajectory.getStateSpace();
  HauserParabolicSmoother parabolicSmoother(_feasibilityCheck,
                                            _timelimit);

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = convertToDynamicPath(_inputTrajectory,
                                          _maxVelocity,
                                          _maxAcceleration);

  parabolicSmoother.doShortcut(*dynamicPath.get(), _rngSeed);

  auto outputTrajectory =
            convertToSpline(*dynamicPath.get(), startTime, stateSpace);

  return outputTrajectory;
}

std::unique_ptr<trajectory::Spline> doBlend(
    const trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit,
    double _blendRadius,
    int _blendIterations)
{
  auto stateSpace = _inputTrajectory.getStateSpace();
  HauserParabolicSmoother parabolicSmoother(_feasibilityCheck,
                                            _timelimit,
                                            _blendRadius,
                                            _blendIterations);

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = convertToDynamicPath(_inputTrajectory,
                                          _maxVelocity,
                                          _maxAcceleration);

  parabolicSmoother.doBlend(*dynamicPath.get());

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
    aikido::util::RNG::result_type _rngSeed)
{
  auto stateSpace = _inputTrajectory.getStateSpace();
  HauserParabolicSmoother parabolicSmoother(_feasibilityCheck,
                                            _timelimit,
                                            _blendRadius,
                                            _blendIterations);

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = convertToDynamicPath(_inputTrajectory,
                                          _maxVelocity,
                                          _maxAcceleration);

  parabolicSmoother.doShortcut(*dynamicPath.get(), _rngSeed);
  parabolicSmoother.doBlend(*dynamicPath.get());

  auto outputTrajectory =
                convertToSpline(*dynamicPath.get(), startTime, stateSpace);

  return outputTrajectory;
}


} // namespace parabolic
} // namespace planner
} // namespace aikido
