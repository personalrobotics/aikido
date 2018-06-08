#ifndef AIKIDO_PLANNER_KINODYNAMIC_KINODYNAMICTIMER_HPP_
#define AIKIDO_PLANNER_KINODYNAMIC_KINODYNAMICTIMER_HPP_

#include <Eigen/Dense>
#include "aikido/planner/TrajectoryPostProcessor.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace kinodynamic {

/// Computes the time-optimal timing of a trajectory consisting of a sequence
/// Geodesic interpolations between states under velocity and acceleration
/// bounds. The output is a parabolic spline, encoded in cubic polynomials,
/// that \b exactly follows the input path.
///
/// The output trajectory consists of a sequence of trapezoidal velocity
/// profiles that implement bang-bang control. This trajectory must stop at
/// each waypoint that introduces a velocity discontinuity to satisfy finite
/// acceleration bounds. You should consider using a blending or smoothing
/// algorithm, which does \b not follow the exact input path, if this behavior
/// is undesirable.
///
/// This function curently only supports \c RealVector, \c SO2, and compound
/// state spaces of those types. Additionally, this function requires that
/// \c _inputTrajectory to be interpolated using a \c GeodesicInterpolator.
///
/// \param _inputTrajectory input piecewise Geodesic trajectory
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \return time optimal trajectory that satisfies acceleration constraints
std::unique_ptr<aikido::trajectory::Spline> computeKinodynamicTiming(
    const aikido::trajectory::Interpolated& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    double maxDeviation = 1e-2,
    double timeStep = 0.1);

std::unique_ptr<aikido::trajectory::Spline> computeKinodynamicTiming(
    const aikido::trajectory::Spline& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    double maxDeviation = 1e-2,
    double timeStep = 0.1);


/// Class for performing time-optimal trajectory retiming following subject to 
/// velocity and acceleration limits.
class KinodynamicTimer : public aikido::planner::TrajectoryPostProcessor
{
public:
  /// \param _velocityLimits Maximum velocity for each dimension.
  /// \param _accelerationLimits Maximum acceleration for each dimension.
  KinodynamicTimer(
      const Eigen::VectorXd& velocityLimits,
      const Eigen::VectorXd& accelerationLimits,
      double maxDeviation,
      double timeStep);

  /// Performs parabolic retiming on an input trajectory.
  /// \copydoc TrajectoryPostProcessor::postprocess
  std::unique_ptr<aikido::trajectory::Spline> postprocess(
      const aikido::trajectory::Interpolated& inputTraj,
      const aikido::common::RNG& rng,
      const aikido::constraint::TestablePtr& constraint = nullptr) override;

  /// Performs parabolic retiming on an input *spline* trajectory.
  /// \copydoc TrajectoryPostProcessor::postprocess
  std::unique_ptr<aikido::trajectory::Spline> postprocess(
      const aikido::trajectory::Spline& inputTraj,
      const aikido::common::RNG& rng,
      const aikido::constraint::TestablePtr& constraint = nullptr) override;

  const Eigen::VectorXd& getVelocityLimits() const;
   
  const Eigen::VectorXd& getAccelerationLimits() const;

  void setVelocityLimits(const Eigen::VectorXd& velocityLimits);
  
  void setAccelerationLimits(const Eigen::VectorXd& accelerationLimits);

  double getTimeStep() const;

  void setTimeStep(double timeStep);

  double getMaxDeviation() const;

  void setMaxDeviation(double maxDeviation);

private:
  /// Set to the value of \c _velocityLimits.
  Eigen::VectorXd mVelocityLimits;

  /// Set to the value of \c _accelerationLimits.
  Eigen::VectorXd mAccelerationLimits;

  double mMaxDeviation;

  double mTimeStep;
};

} // namespace kinodynamic
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_KINODYNAMIC_KINODYNAMICTIMER_HPP_
