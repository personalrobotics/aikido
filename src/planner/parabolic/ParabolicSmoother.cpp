#include "aikido/planner/parabolic/ParabolicSmoother.hpp"

#include <cassert>
#include <set>

#include "aikido/common/Spline.hpp"
#include "aikido/common/memory.hpp"
#include "aikido/planner/parabolic/ParabolicTimer.hpp"

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

  if (!dynamicPath)
  {
    return nullptr;
  }
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

  if (!dynamicPath)
  {
    return nullptr;
  }
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

  if (!dynamicPath)
  {
    return nullptr;
  }
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

//==============================================================================
ParabolicSmoother::ParabolicSmoother(
    const Eigen::VectorXd& _velocityLimits,
    const Eigen::VectorXd& _accelerationLimits,
    bool _enableShortcut,
    bool _enableBlend,
    double _shortcutTimelimit,
    double _blendRadius,
    int _blendIterations,
    double _feasibilityCheckResolution,
    double _feasibilityApproxTolerance)
  : mFeasibilityCheckResolution{_feasibilityCheckResolution}
  , mFeasibilityApproxTolerance{_feasibilityApproxTolerance}
  , mVelocityLimits{_velocityLimits}
  , mAccelerationLimits{_accelerationLimits}
  , mEnableShortcut{_enableShortcut}
  , mEnableBlend{_enableBlend}
  , mShortcutTimelimit{_shortcutTimelimit}
  , mBlendRadius{_blendRadius}
  , mBlendIterations{_blendIterations}
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> ParabolicSmoother::postprocess(
    const aikido::trajectory::Interpolated& _inputTraj,
    const aikido::common::RNG& _rng,
    const aikido::constraint::TestablePtr& _collisionTestable)
{
  // Get timed trajectory for arm
  auto timedTrajectory = computeParabolicTiming(
      _inputTraj, mVelocityLimits, mAccelerationLimits);

  if (!timedTrajectory)
  {
    return timedTrajectory;
  }
  auto shortcutOrBlendTrajectory
      = handleShortcutOrBlend(*timedTrajectory, _rng, _collisionTestable);
  if (shortcutOrBlendTrajectory)
    return shortcutOrBlendTrajectory;

  return timedTrajectory;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> ParabolicSmoother::postprocess(
    const aikido::trajectory::Spline& _inputTraj,
    const aikido::common::RNG& _rng,
    const aikido::constraint::TestablePtr& _collisionTestable)
{
  // Get timed trajectory for arm
  auto timedTrajectory = computeParabolicTiming(
      _inputTraj, mVelocityLimits, mAccelerationLimits);

  if (!timedTrajectory)
  {
    return timedTrajectory;
  }

  auto shortcutOrBlendTrajectory
      = handleShortcutOrBlend(*timedTrajectory, _rng, _collisionTestable);
  if (shortcutOrBlendTrajectory)
    return shortcutOrBlendTrajectory;

  return timedTrajectory;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline>
ParabolicSmoother::handleShortcutOrBlend(
    const aikido::trajectory::Spline& _inputTraj,
    const aikido::common::RNG& _rng,
    const aikido::constraint::TestablePtr& _collisionTestable)
{
  if (!_collisionTestable)
    throw std::invalid_argument(
        "_collisionTestable passed to ParabolicSmoother is nullptr.");

  if (mEnableShortcut && mEnableBlend)
  {
    return doShortcutAndBlend(
        _inputTraj,
        _collisionTestable,
        mVelocityLimits,
        mAccelerationLimits,
        *_rng.clone(),
        mShortcutTimelimit,
        mBlendRadius,
        mBlendIterations,
        mFeasibilityCheckResolution,
        mFeasibilityApproxTolerance);
  }
  else if (mEnableShortcut)
  {
    return doShortcut(
        _inputTraj,
        _collisionTestable,
        mVelocityLimits,
        mAccelerationLimits,
        *_rng.clone(),
        mShortcutTimelimit,
        mFeasibilityCheckResolution,
        mFeasibilityApproxTolerance);
  }
  else if (mEnableBlend)
  {
    return doBlend(
        _inputTraj,
        _collisionTestable,
        mVelocityLimits,
        mAccelerationLimits,
        mBlendRadius,
        mBlendIterations,
        mFeasibilityCheckResolution,
        mFeasibilityApproxTolerance);
  }

  return nullptr;
}

} // namespace parabolic
} // namespace planner
} // namespace aikido
