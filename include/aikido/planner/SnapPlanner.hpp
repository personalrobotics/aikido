#ifndef AIKIDO_PLANNER_SNAPPLANNER_HPP_
#define AIKIDO_PLANNER_SNAPPLANNER_HPP_

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/PlanToConfiguration.hpp"
#include "aikido/planner/Planner.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

class SnapPlanner : public Planner
{
public:
  /// Constructor
  SnapPlanner();

  /// Plan for PlanToConfiguration problem.
  ///
  /// The planner returns success if the resulting trajectory satisfies
  /// constraint at some resolution and failure (returning \c nullptr)
  /// otherwise. The reason for the failure is stored in the \c result output
  /// parameter.
  ///
  /// \param problem Planning problem.
  /// \param[out] result Information about success or failure.
  /// \return Trajectory or \c nullptr if planning failed.
  /// \throw If \c problem is not PlanToConfiguration.
  /// \throw If \c result is not PlanToConfiguration::Result.
  trajectory::InterpolatedPtr planToConfiguration(
      const Problem* problem, Problem::Result* result = nullptr);

protected:
  PlanningFunctionMap& getPlanningFunctionMap() override;

  static bool mIsRegisteredPlanningFunctions;
  static PlanningFunctionMap mPlanningFunctionMap;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_SNAPPLANNER_HPP_
