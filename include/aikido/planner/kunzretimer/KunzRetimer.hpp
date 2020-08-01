#ifndef AIKIDO_PLANNER_KUNZRETIMER_KUNZRETIMER_HPP_
#define AIKIDO_PLANNER_KUNZRETIMER_KUNZRETIMER_HPP_

#include <Eigen/Dense>

#include "aikido/planner/TrajectoryPostProcessor.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace kunzretimer {

constexpr double DEFAULT_MAX_DEVIATION = 1e-2;
constexpr double DEFAULT_TIME_STEP = 0.1;

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
/// \param[in] inputTrajectory Input piecewise Geodesic trajectory
/// \param[in] maxVelocity Maximum velocity for each dimension
/// \param[in] maxAcceleration Maximum acceleration for each dimension
/// \param[in] maxDeviation Maximum deviation from a waypoint in doing circular
/// blending around the waypoint
/// \param[in] timeStep Time step in following the path
/// \return Time optimal trajectory that satisfies velocity and acceleration
/// constraints
std::unique_ptr<aikido::trajectory::Spline> computeKunzTiming(
    const aikido::trajectory::Interpolated& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    double maxDeviation = DEFAULT_MAX_DEVIATION,
    double timeStep = DEFAULT_TIME_STEP);

/// Class for performing time-optimal trajectory retiming following subject to
/// velocity and acceleration limits.
class KunzRetimer : public aikido::planner::TrajectoryPostProcessor
{
public:
  /// Kunz postprocessor parameters.
  struct Params
  {
    /// \param[in] _maxDeviation Maximum deviation in circular blending (in
    /// configuration space).
    /// \param[in] _timeStep Time step in following the path (in seconds).
    Params(
        double _maxDeviation = DEFAULT_MAX_DEVIATION,
        double _timeStep = DEFAULT_TIME_STEP)
      : mMaxDeviation(_maxDeviation), mTimeStep(_timeStep)
    {
      // Do nothing.
    }

    double mMaxDeviation;
    double mTimeStep;
  };

  /// \param[in] velocityLimits Maximum velocity for each dimension.
  /// \param[in] accelerationLimits Maximum acceleration for each dimension.
  /// \param[in] maxDeviation Maximum deviation in circular blending (in
  /// configuration space).
  /// \param[in] timeStep Time step in following the path (in seconds).
  KunzRetimer(
      const Eigen::VectorXd& velocityLimits,
      const Eigen::VectorXd& accelerationLimits,
      double maxDeviation = DEFAULT_MAX_DEVIATION,
      double timeStep = DEFAULT_TIME_STEP);

  /// \param[in] velocityLimits Maximum velocity for each dimension.
  /// \param[in] accelerationLimits Maximum acceleration for each dimension.
  /// \param[in] params Postprocessor parameters.
  KunzRetimer(
      const Eigen::VectorXd& velocityLimits,
      const Eigen::VectorXd& accelerationLimits,
      const Params& params);

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

} // namespace kunzretimer
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_KUNZRETIMER_KUNZRETIMER_HPP_
