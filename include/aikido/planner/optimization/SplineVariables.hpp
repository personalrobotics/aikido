#ifndef AIKIDO_PLANNER_OPTIMIZATION_SPLINEVARIABLES_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_SPLINEVARIABLES_HPP_

#include "aikido/planner/optimization/TrajectoryOptimizationVariables.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class SplineVariables : public TrajectoryVariables
{
public:
  /// Constructor
  explicit SplineVariables(const trajectory::Spline& splineToClone);

  // Documentation inherited.
  const trajectory::Trajectory& getTrajectory() const override;

  /// Returns const Spline
  ///
  /// We only provide const version of this function because we don't intend
  /// the structure and parameters of spline to be changed by the caller.
  const trajectory::Spline& getSpline() const;

  /// Returns the dimension of this trajectory optimization variables.
  std::size_t getDimension() const;

protected:
  /// Update the dimension of this trajectory optimization variables.
  virtual void updateDimension() = 0;

  /// Spline
  trajectory::Spline mSpline;
  // Note: This class is intended to only change the spline parameters (i.e.,
  // values of segment coefficients or duration) so DO NOT change the structures
  // (i.e., number of segments or shape of coefficients). This is important to
  // keep mDimension validate.

  /// Dimension of this trajectory optimization variables.
  ///
  /// The dimension should be decided by concrete classes of this class by
  /// calling updateDimension()
  std::size_t mDimension;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_SPLINEVARIABLES_HPP_
