#include <stdexcept>
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"
#include "aikido/planner/parabolic/ParabolicTimer.hpp"
#include "aikido/planner/postprocessor/SmoothTrajectoryPostProcessor.hpp"

namespace aikido {
namespace planner {
namespace postprocessor {

using aikido::trajectory::Spline;
using aikido::trajectory::InterpolatedPtr;
using aikido::common::RNG;

SmoothTrajectoryPostProcessor::SmoothTrajectoryPostProcessor(
    aikido::statespace::StateSpacePtr _space,
    double _smootherFeasibilityCheckResolution,
    double _smootherFeasibilityApproxTolerance,
    const Eigen::VectorXd& _velocityLimits,
    const Eigen::VectorXd& _accelerationLimits,
    const aikido::constraint::TestablePtr& _collisionTestable,
    bool _enableShortcut,
    bool _enableBlend,
    double _shortcutTimelimit,
    double _blendRadius,
    int _blendIterations)
  : mSpace{std::move(_space)}
  , mSmootherFeasibilityCheckResolution{_smootherFeasibilityCheckResolution}
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

//==============================================================================
std::unique_ptr<Spline> SmoothTrajectoryPostProcessor::postprocess(
    const InterpolatedPtr& _inputTraj, RNG* _rng)
{
  using aikido::planner::parabolic::computeParabolicTiming;
  using aikido::planner::parabolic::doShortcut;
  using aikido::planner::parabolic::doBlend;
  using aikido::planner::parabolic::doShortcutAndBlend;

  if (!_rng)
    throw std::invalid_argument("Passed nullptr _rng to SmoothTrajectoryPostProcessor::postprocess");
  if (!_inputTraj)
    throw std::invalid_argument("Passed nullptr _inputTraj to SmoothTrajectoryPostProcessor::postprocess");


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
} // namespace postprocessor
} // namespace planner
} // namespace aikido
