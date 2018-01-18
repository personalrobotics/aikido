#ifndef AIKIDO_PLANNER_POSTPROCESSOR_SMOOTHTRAJECTORYPOSTPROCESSOR_HPP_
#define AIKIDO_PLANNER_POSTPROCESSOR_SMOOTHTRAJECTORYPOSTPROCESSOR_HPP_

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"
#include "aikido/planner/postprocessor/TrajectoryPostProcessor.hpp"

namespace aikido {
namespace planner {
namespace postprocessor {

class SmoothTrajectoryPostProcessor : public TrajectoryPostProcessor
{
public:
  /// Class for performing parabolic smoothing on trajectories
  /// \param _space pointer to statespace trajectories correspond to.
  /// \param _smootherFeasibilityCheckResolution the resolution in discretizing
  /// a segment in checking the feasibility of the segment.
  /// \param _smootherFeasibilityApproxTolerance this tolerance is used in a
  /// piecewise linear discretization that deviates no more than
  /// \c _smootherFeasibilityApproxTolerance from the parabolic ramp along any
  /// axis, and then checks for configuration and segment feasibility along that
  /// piecewise linear path.
  /// \param _velocityLimits maximum velocity for each dimension.
  /// \param _accelerationLimits maximum acceleration for each dimension.
  /// \param _collisionTestable Check whether a position is feasible.
  /// \param _enableShortcut Whether shortcutting is used in smoothing.
  /// \param _enableBlend Whether blending is used in smoothing.
  /// \param _shortcutTimelimit Timelimit for shortcutting. It is ineffective
  /// when _enableShortcut is false.
  /// \param _blendRadius Blend radius for blending. It is ineffective
  /// when _enableBlend is false.
  /// \param _blendIterations Blend iterations for blending. It is
  /// ineffective when _enableBlend is false.
  SmoothTrajectoryPostProcessor(
      aikido::statespace::StateSpacePtr _space,
      const Eigen::VectorXd& _velocityLimits,
      const Eigen::VectorXd& _accelerationLimits,
      const aikido::constraint::TestablePtr& _collisionTestable,
      bool _enableShortcut = true,
      bool _enableBlend = true,
      double _shortcutTimelimit = aikido::planner::parabolic::DEFAULT_TIMELIMT,
      double _blendRadius = aikido::planner::parabolic::DEFAULT_BLEND_RADIUS,
      int _blendIterations
      = aikido::planner::parabolic::DEFAULT_BLEND_ITERATIONS,
      double _smootherFeasibilityCheckResolution
      = aikido::planner::parabolic::DEFAULT_CHECK_RESOLUTION,
      double _smootherFeasibilityApproxTolerance
      = aikido::planner::parabolic::DEFAULT_TOLERANCE);

  // Documentation inherited.
  std::unique_ptr<aikido::trajectory::Spline> postprocess(
      const aikido::trajectory::InterpolatedPtr& _inputTraj,
      const aikido::common::RNG* _rng) override;

private:
  aikido::statespace::StateSpacePtr mSpace;
  double mSmootherFeasibilityCheckResolution;
  double mSmootherFeasibilityApproxTolerance;
  const Eigen::VectorXd mVelocityLimits;
  const Eigen::VectorXd mAccelerationLimits;
  aikido::constraint::TestablePtr mCollisionTestable;

  bool mEnableShortcut;
  bool mEnableBlend;
  double mShortcutTimelimit;
  double mBlendRadius;
  int mBlendIterations;
};

} // namespace postprocessor
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_POSTPROCESSOR_SMOOTHTRAJECTORYPOSTPROCESSOR_HPP_
