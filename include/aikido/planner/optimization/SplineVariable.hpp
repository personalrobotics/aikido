#ifndef AIKIDO_PLANNER_OPTIMIZATION_SPLINEVARIABLES_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_SPLINEVARIABLES_HPP_

#include "aikido/planner/optimization/TrajectoryVariable.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class SplineVariable : public TrajectoryVariable
{
public:
  /// Constructor
  explicit SplineVariable(const trajectory::Spline& splineToClone);

  // Documentation inherited.
  const trajectory::Trajectory& getTrajectory() const override;

  /// Returns const Spline
  ///
  /// We only provide const version of this function because we don't intend
  /// the structure and parameters of spline to be changed by the caller.
  const trajectory::Spline& getSpline() const;

protected:
  /// Spline
  trajectory::Spline mSpline;
  // Note: This class is intended to only change the spline parameters (i.e.,
  // values of segment coefficients or duration) so DO NOT change the structures
  // (i.e., number of segments or shape of coefficients). This is important to
  // keep mDimension validate.
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_SPLINEVARIABLES_HPP_
