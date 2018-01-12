#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpaceSaver.hpp>
#include <aikido/trajectory/postprocess.hpp>
#include <dart/common/StlHelpers.hpp>

namespace aikido {
namespace trajectory {
namespace postprocess {

using aikido::constraint::TestablePtr;
using aikido::constraint::createProjectableBounds;
using aikido::constraint::createSampleableBounds;
using aikido::constraint::createTestableBounds;
using aikido::distance::createDistanceMetric;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::statespace::dart::MetaSkeletonStateSpaceSaver;
using aikido::trajectory::Interpolated;
using aikido::trajectory::InterpolatedPtr;
using aikido::common::RNG;
using aikido::trajectory::Spline;

using dart::common::make_unique;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;

//==============================================================================
std::pair<std::unique_ptr<Interpolated>, bool>
simplifyTrajectory(const MetaSkeletonStateSpacePtr &_space,
                   const TestablePtr &_collisionTestable, double _timeOut,
                   size_t _maxEmptySteps, const InterpolatedPtr &_originalTraj,
                   std::unique_ptr<RNG> _rng, double _collisionResolution) {
  using aikido::planner::ompl::simplifyOMPL;

  auto simplifiedPair = simplifyOMPL(
      _space, std::make_shared<GeodesicInterpolator>(_space),
      createDistanceMetric(_space),
      createSampleableBounds(_space, std::move(_rng)), _collisionTestable,
      createTestableBounds(_space), createProjectableBounds(_space),
      _collisionResolution, _timeOut, _maxEmptySteps, _originalTraj);

  return simplifiedPair;
}

//==============================================================================
std::unique_ptr<Spline>
timeTrajectory(const InterpolatedPtr &_inputTraj,
               const Eigen::VectorXd &_velocityLimits,
               const Eigen::VectorXd &_accelerationLimits) {
  using aikido::planner::parabolic::computeParabolicTiming;
  using aikido::statespace::dart::MetaSkeletonStateSpace;

  MetaSkeletonStateSpacePtr space =
      std::dynamic_pointer_cast<MetaSkeletonStateSpace>(
          _inputTraj->getStateSpace());

  // Get timed trajectory for arm
  return computeParabolicTiming(*_inputTraj, _velocityLimits,
                                _accelerationLimits);
}

//==============================================================================
std::unique_ptr<Spline> smoothTrajectory(
    const InterpolatedPtr &_inputTraj, bool _enableShortcut, bool _enableBlend,
    const TestablePtr &_collisionTestable, std::unique_ptr<RNG> _rng,
    const Eigen::VectorXd &_velocityLimits,
    const Eigen::VectorXd &_accelerationLimits,
    double _smootherFeasibilityCheckResolution,
    double _smootherFeasibilityApproxTolerance, double _shortcutTimelimit,
    double _blendRadius, int _blendIterations) {
  using aikido::statespace::dart::MetaSkeletonStateSpace;
  using aikido::planner::parabolic::computeParabolicTiming;
  using aikido::planner::parabolic::doShortcut;
  using aikido::planner::parabolic::doBlend;
  using aikido::planner::parabolic::doShortcutAndBlend;

  MetaSkeletonStateSpacePtr space =
      std::dynamic_pointer_cast<MetaSkeletonStateSpace>(
          _inputTraj->getStateSpace());
  auto saver = MetaSkeletonStateSpaceSaver(space);
  DART_UNUSED(saver);

  // Get timed trajectory for arm
  auto timedTrajectory =
      computeParabolicTiming(*_inputTraj, _velocityLimits, _accelerationLimits);

  if (_enableShortcut && _enableBlend) {
    return doShortcutAndBlend(
        *timedTrajectory, _collisionTestable, _velocityLimits,
        _accelerationLimits, *_rng, _shortcutTimelimit, _blendRadius,
        _blendIterations, _smootherFeasibilityCheckResolution,
        _smootherFeasibilityApproxTolerance);
  } else if (_enableShortcut) {
    return doShortcut(*timedTrajectory, _collisionTestable, _velocityLimits,
                      _accelerationLimits, *_rng, _shortcutTimelimit,
                      _smootherFeasibilityCheckResolution,
                      _smootherFeasibilityApproxTolerance);
  } else if (_enableBlend) {
    return doBlend(*timedTrajectory, _collisionTestable, _velocityLimits,
                   _accelerationLimits, _blendRadius, _blendIterations,
                   _smootherFeasibilityCheckResolution,
                   _smootherFeasibilityApproxTolerance);
  }

  return timedTrajectory;
}

} // namespace postprocess
} // namespace trajectory
} // namespace aikido
