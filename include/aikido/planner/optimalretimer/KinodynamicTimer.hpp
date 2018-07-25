#ifndef AIKIDO_PLANNER_KINODYNAMIC_KINODYNAMICTIMER_HPP_
#define AIKIDO_PLANNER_KINODYNAMIC_KINODYNAMICTIMER_HPP_

#include <Eigen/Dense>
#include "aikido/planner/TrajectoryPostProcessor.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace optimalretimer {

/// Computes the time-optimal timing of a trajectory consisting of a sequence
/// Geodesic interpolations between states under velocity and acceleration
/// bounds. The output is a parabolic spline, encoded in cubic polynomials.
/// It firstly preprocesses a non-differentiable path to a differentiable one
/// by adding circular blends; and then \b exactly follows the preprocessed
/// path.
///
/// The output trajectory consists of a sequence of trapezoidal velocity
/// profiles that implement bang-bang control. This function curently only
/// supports \c RealVector, \c SO2, and compound state spaces of those types.
/// Additionally, this function requires that \c inputTrajectory to be
/// interpolated using a \c GeodesicInterpolator.
///
/// \param[in] inputTrajectory input piecewise Geodesic trajectory
/// \param[in] maxVelocity maximum velocity for each dimension
/// \param[in] maxAcceleration maximum acceleration for each dimension
/// \param[in] maxDeviation maximum deviation from a waypoint in doing circular
/// blending around the waypoint
/// \param[in] timeStep time step in following the path
/// \return time optimal trajectory that satisfies velocity and acceleration constraints
std::unique_ptr<aikido::trajectory::Spline> computeKinodynamicTiming(
    const aikido::trajectory::Interpolated& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    double maxDeviation = 1e-2,
    double timeStep = 0.1);

/// Computes the time-optimal timing of a spline trajectory under velocity
/// and acceleration bounds. The output is another parabolic spline, encoded
/// in cubic polynomials. In retiming, the input trajectory is used as a
/// piecewise Geodesic trajectory (only the geometry of the input trajectory
/// is considered; and the velocity information is ignored. It firstly
/// preprocesses a non-differentiable path to a differentiable one by adding
/// circular blends; and then \b exactly follows the preprocessed path.
///
/// The output trajectory consists of a sequence of trapezoidal velocity
/// profiles that implement bang-bang control. This function curently only
/// supports \c RealVector, \c SO2, and compound state spaces of those types.
/// Additionally, this function requires that \c inputTrajectory to be
/// a spline (only the geometry of the trajectory is considered in retiming).
///
/// \param[in] inputTrajectory input piecewise Geodesic trajectory
/// \param[in] maxVelocity maximum velocity for each dimension
/// \param[in] maxAcceleration maximum acceleration for each dimension
/// \param[in] maxDeviation maximum deviation from a waypoint in doing circular
/// blending around the waypoint
/// \param[in] timeStep time step in following the path
/// \return time optimal trajectory that satisfies acceleration constraints
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
  /// \param[in] velocityLimits Maximum velocity for each dimension.
  /// \param[in] accelerationLimits Maximum acceleration for each dimension.
  /// \param[in] maxDeviation Maximum deviation in circular blending
  /// \param[in] timeStep Time step in following the path
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

  /// Returns the velocity limits of the dimensions
  const Eigen::VectorXd& getVelocityLimits() const;

  /// Returns the acceleration limits of the dimensions
  const Eigen::VectorXd& getAccelerationLimits() const;

  /// Sets the velocity limits of the dimensions
  void setVelocityLimits(const Eigen::VectorXd& velocityLimits);

  /// Sets the acceleration limits of the dimensions
  void setAccelerationLimits(const Eigen::VectorXd& accelerationLimits);

  /// Returns the time step in following a path
  double getTimeStep() const;

  /// Sets the time step in following a path
  void setTimeStep(double timeStep);

  /// Returns the max deviation of circular blending
  double getMaxDeviation() const;

  /// Sets the max deviation of circular blending
  void setMaxDeviation(double maxDeviation);

private:
  /// Set to the value of \c velocityLimits.
  Eigen::VectorXd mVelocityLimits;

  /// Set to the value of \c accelerationLimits.
  Eigen::VectorXd mAccelerationLimits;

  /// Set to the value of \c maxDeviation
  double mMaxDeviation;

  /// Set to the value of \c timeStep
  double mTimeStep;
};

} // namespace optimal_retimer
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_KINODYNAMIC_KINODYNAMICTIMER_HPP_
