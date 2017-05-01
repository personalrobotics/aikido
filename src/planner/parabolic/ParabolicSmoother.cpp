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
    aikido::trajectory::Spline* _inputTrajectory,
    const aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit,
    bool _useVelocity)
{
  auto stateSpace = _inputTrajectory->getStateSpace();
  HauserParabolicSmoother parabolicSmoother(_feasibilityCheck,
                                            _timelimit,
                                            _useVelocity);
  ParabolicRamp::DynamicPath dynamicPath;

  convertToDynamicPath(_inputTrajectory, dynamicPath);

  // Apply the adjoint limits
  dynamicPath.Init(toVector(_maxVelocity), toVector(_maxAcceleration));

  parabolicSmoother.doShortcut(dynamicPath);

  aikido::trajectory::Spline* outputTrajectory =
            convertToSpline(dynamicPath, stateSpace);

  return std::unique_ptr<aikido::trajectory::Spline>(outputTrajectory);
}


std::unique_ptr<trajectory::Spline> doBlend(
    trajectory::Spline* _inputTrajectory,
    const aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit,
    bool _useVelocity,
    double _blendRadius,
    int _blendIterations)
{
  auto stateSpace = _inputTrajectory->getStateSpace();
  HauserParabolicSmoother parabolicSmoother(_feasibilityCheck,
                                            _timelimit,
                                            _useVelocity,
                                            _blendRadius,
                                            _blendIterations);
  ParabolicRamp::DynamicPath dynamicPath;

  convertToDynamicPath(_inputTrajectory, dynamicPath);

  // Apply the adjoint limits
  dynamicPath.Init(toVector(_maxVelocity), toVector(_maxAcceleration));

  parabolicSmoother.doBlend(dynamicPath);

  aikido::trajectory::Spline* outputTrajectory =
              convertToSpline(dynamicPath, stateSpace);

  return std::unique_ptr<aikido::trajectory::Spline>(outputTrajectory);
}

std::unique_ptr<trajectory::Spline> doShortcutAndBlend(
    trajectory::Spline* _inputTrajectory,
    const aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit,
    bool _useVelocity,
    double _blendRadius,
    int _blendIterations)
{
  auto stateSpace = _inputTrajectory->getStateSpace();
  HauserParabolicSmoother parabolicSmoother(_feasibilityCheck,
                                            _timelimit,
                                            _useVelocity,
                                            _blendRadius,
                                            _blendIterations);
  ParabolicRamp::DynamicPath dynamicPath;

  convertToDynamicPath(_inputTrajectory, dynamicPath);

  // Apply the adjoint limits
  dynamicPath.Init(toVector(_maxVelocity), toVector(_maxAcceleration));

  parabolicSmoother.doShortcut(dynamicPath);
  parabolicSmoother.doBlend(dynamicPath);

  aikido::trajectory::Spline* outputTrajectory =
                convertToSpline(dynamicPath, stateSpace);

  return std::unique_ptr<aikido::trajectory::Spline>(outputTrajectory);
}


} // namespace parabolic
} // namespace planner
} // namespace aikido
