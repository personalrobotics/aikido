#include <aikido/trajectory/SmoothTrajectoryPostProcessor.hpp>
#include <aikido/trajectory/postprocess.hpp>

namespace aikido {
namespace trajectory {

using aikido::trajectory::Spline;
using aikido::trajectory::InterpolatedPtr;
using aikido::common::RNG;

SmoothTrajectoryPostProcessor::SmoothTrajectoryPostProcessor(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr _space,
    double _smootherFeasibilityCheckResolution,
    double _smootherFeasibilityApproxTolerance,
    const Eigen::VectorXd &_velocityLimits,
    const Eigen::VectorXd &_accelerationLimits,
    const aikido::constraint::TestablePtr &_collisionTestable,
    bool _enableShortcut, bool _enableBlend, double _shortcutTimelimit,
    double _blendRadius, int _blendIterations)
    : mSpace{_space},
      mSmootherFeasibilityCheckResolution{_smootherFeasibilityCheckResolution},
      mSmootherFeasibilityApproxTolerance{_smootherFeasibilityApproxTolerance},
      mVelocityLimits{_velocityLimits},
      mAccelerationLimits{_accelerationLimits},
      mCollisionTestable{_collisionTestable}, mEnableShortcut{_enableShortcut},
      mEnableBlend{_enableBlend}, mShortcutTimelimit{_shortcutTimelimit},
      mBlendRadius{_blendRadius}, mBlendIterations{_blendIterations} {
  // Do nothing
}

std::unique_ptr<Spline>
SmoothTrajectoryPostProcessor::postprocess(const InterpolatedPtr &_inputTraj,
                                           RNG *_rng) {

  auto timedTrajectory = postprocess::smoothTrajectory(
      _inputTraj, mEnableShortcut, mEnableBlend, mCollisionTestable,
      _rng->clone(), mVelocityLimits, mAccelerationLimits,
      mSmootherFeasibilityCheckResolution, mSmootherFeasibilityApproxTolerance,
      mShortcutTimelimit, mBlendRadius, mBlendIterations);

  return timedTrajectory;
}

}
}
