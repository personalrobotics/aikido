#ifndef AIKIDO_PLANNER_POSTPROCESSOR_SMOOTHTRAJECTORYPOSTPROCESSOR_HPP_
#define AIKIDO_PLANNER_POSTPROCESSOR_SMOOTHTRAJECTORYPOSTPROCESSOR_HPP_

#include "../../constraint/Testable.hpp"
#include "TrajectoryPostProcessor.hpp"

namespace aikido {
namespace planner {
namespace postprocessor {

class SmoothTrajectoryPostProcessor : public TrajectoryPostProcessor
{
public:
  /// Perform parabolic smoothing on trajectory
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
      double _smootherFeasibilityCheckResolution,
      double _smootherFeasibilityApproxTolerance,
      const Eigen::VectorXd& _velocityLimits,
      const Eigen::VectorXd& _accelerationLimits,
      const aikido::constraint::TestablePtr& _collisionTestable,
      bool _enableShortcut,
      bool _enableBlend,
      double _shortcutTimelimit,
      double _blendRadius,
      int _blendIterations);

  // Documentation inherited.
  virtual std::unique_ptr<aikido::trajectory::Spline> postprocess(
      const aikido::trajectory::InterpolatedPtr& _inputTraj,
      aikido::common::RNG* _rng) override;

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

} // postprocessor
} // planner
} // aikido

#endif // AIKIDO_PLANNER_POSTPROCESSOR_SMOOTHTRAJECTORYPOSTPROCESSOR_HPP_
