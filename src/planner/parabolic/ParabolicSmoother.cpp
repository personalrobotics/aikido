#include <cassert>
#include <set>
#include <dart/common/StlHelpers.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
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
    aikido::common::RNG& _rng,
    double _timelimit,
    double _checkResolution,
    double _tolerance)
{
  auto stateSpace = _inputTrajectory.getStateSpace();

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = detail::convertToDynamicPath(
      _inputTrajectory, _maxVelocity, _maxAcceleration);

  detail::doShortcut(
      *dynamicPath,
      _feasibilityCheck,
      _timelimit,
      _checkResolution,
      _tolerance,
      _rng);

  auto outputTrajectory
      = detail::convertToSpline(*dynamicPath, startTime, stateSpace);

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

  auto outputTrajectory
      = detail::convertToSpline(*dynamicPath, startTime, stateSpace);

  return outputTrajectory;
}

std::unique_ptr<trajectory::Spline> doShortcutAndBlend(
    const trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    aikido::common::RNG& _rng,
    double _timelimit,
    double _blendRadius,
    int _blendIterations,
    double _checkResolution,
    double _tolerance)
{
  auto stateSpace = _inputTrajectory.getStateSpace();

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = detail::convertToDynamicPath(
      _inputTrajectory, _maxVelocity, _maxAcceleration);

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

  auto outputTrajectory
      = detail::convertToSpline(*dynamicPath, startTime, stateSpace);

  return outputTrajectory;
}

SmoothTrajectoryPostProcessor::SmoothTrajectoryPostProcessor(
    const Eigen::VectorXd& _velocityLimits,
    const Eigen::VectorXd& _accelerationLimits,
    const aikido::constraint::TestablePtr& _collisionTestable,
    bool _enableShortcut,
    bool _enableBlend,
    double _shortcutTimelimit,
    double _blendRadius,
    int _blendIterations,
    double _smootherFeasibilityCheckResolution,
    double _smootherFeasibilityApproxTolerance)
  : mSmootherFeasibilityCheckResolution{_smootherFeasibilityCheckResolution}
  , mSmootherFeasibilityApproxTolerance{_smootherFeasibilityApproxTolerance}
  , mVelocityLimits{_velocityLimits}
  , mAccelerationLimits{_accelerationLimits}
  , mCollisionTestable{_collisionTestable}
  , mEnableShortcut{_enableShortcut}
  , mEnableBlend{_enableBlend}
  , mShortcutTimelimit{_shortcutTimelimit}
  , mBlendRadius{_blendRadius}
  , mBlendIterations{_blendIterations}
{
  // Do nothing
}

std::unique_ptr<aikido::trajectory::Spline>
SmoothTrajectoryPostProcessor::postprocess(
    const aikido::trajectory::InterpolatedPtr& _inputTraj,
    const aikido::common::RNG* _rng)
{
  if (!_rng)
    throw std::invalid_argument(
        "Passed nullptr _rng to SmoothTrajectoryPostProcessor::postprocess");
  if (!_inputTraj)
    throw std::invalid_argument(
        "Passed nullptr _inputTraj to "
        "SmoothTrajectoryPostProcessor::postprocess");

  // Get timed trajectory for arm
  auto timedTrajectory = computeParabolicTiming(
      *_inputTraj, mVelocityLimits, mAccelerationLimits);

  if (mEnableShortcut && mEnableBlend)
  {
    return doShortcutAndBlend(
        *timedTrajectory,
        mCollisionTestable,
        mVelocityLimits,
        mAccelerationLimits,
        *_rng->clone(),
        mShortcutTimelimit,
        mBlendRadius,
        mBlendIterations,
        mSmootherFeasibilityCheckResolution,
        mSmootherFeasibilityApproxTolerance);
  }
  else if (mEnableShortcut)
  {
    return doShortcut(
        *timedTrajectory,
        mCollisionTestable,
        mVelocityLimits,
        mAccelerationLimits,
        *_rng->clone(),
        mShortcutTimelimit,
        mSmootherFeasibilityCheckResolution,
        mSmootherFeasibilityApproxTolerance);
  }
  else if (mEnableBlend)
  {
    return doBlend(
        *timedTrajectory,
        mCollisionTestable,
        mVelocityLimits,
        mAccelerationLimits,
        mBlendRadius,
        mBlendIterations,
        mSmootherFeasibilityCheckResolution,
        mSmootherFeasibilityApproxTolerance);
  }

  return timedTrajectory;
}

} // namespace parabolic
} // namespace planner
} // namespace aikido
