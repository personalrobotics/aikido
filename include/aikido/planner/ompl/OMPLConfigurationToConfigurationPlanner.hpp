#ifndef AIKIDO_PLANNER_OMPL_OMPLCONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
#define AIKIDO_PLANNER_OMPL_OMPLCONFIGURATIONTOCONFIGURATIONPLANNER_HPP_

#include "aikido/planner/ConfigurationToConfiguration.hpp"
#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"

#include "aikido/planner/ompl/GeometricStateSpace.hpp"


#include "../../constraint/Projectable.hpp"
#include "../../constraint/Sampleable.hpp"
#include "../../constraint/Testable.hpp"
#include "../../distance/DistanceMetric.hpp"
#include "../../statespace/StateSpace.hpp"




#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalRegion.h>

#include <ompl/geometric/PathSimplifier.h>

namespace aikido {
namespace planner {
namespace ompl {

template<class PlannerType>
class OMPLConfigurationToConfigurationPlanner
    : public aikido::planner::ConfigurationToConfigurationPlanner
{
public:
  /// Constructor.
  ///
  /// \param[in] planner The OMPL planner to use.
  /// \param[in] stateSpace State space that this planner associated with.
  /// \param[in] interpolator Interpolator used to produce the output
  /// trajectory. If nullptr is passed in, GeodesicInterpolator is used by
  /// default.
  OMPLConfigurationToConfigurationPlanner(
      statespace::ConstStateSpacePtr stateSpace,
      statespace::ConstInterpolatorPtr interpolator,
      distance::DistanceMetricPtr dmetric,
      constraint::SampleablePtr sampler,
      constraint::TestablePtr validityConstraint,
      constraint::TestablePtr boundsConstraint,
      constraint::ProjectablePtr boundsProjector,
      double maxDistanceBtwValidityChecks);

  /// Plans a trajectory from start state to goal state by using an interpolator
  /// to interpolate between them.
  ///
  /// The planner returns success if the resulting trajectory satisfies
  /// constraint at some resolution and failure (returning \c nullptr)
  /// otherwise. The reason for the failure is stored in the \c result output
  /// parameter.
  ///
  /// \param[in] problem Planning problem.
  /// \param[out] result Information about success or failure.
  /// \return Trajectory or \c nullptr if planning failed.
  /// \throw If \c problem is not ConfigurationToConfiguration.
  /// \throw If \c result is not ConfigurationToConfiguration::Result.
  trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr) override;

  /// Returns the underlying OMPL planner used.
  ::ompl::base::PlannerPtr getOMPLPlanner();

  /// Sets interpolator used to produce the output trajectory.
  void setInterpolator(statespace::ConstInterpolatorPtr interpolator);

  /// Returns the interpolator used to produce the output trajectory.
  statespace::ConstInterpolatorPtr getInterpolator() const;

protected:
  /// Pointer to the underlying OMPL Planner.
  ::ompl::base::PlannerPtr mPlanner;

  /// Interpolator used to produce the output trajectory.
  statespace::ConstInterpolatorPtr mInterpolator;
};

} // namespace ompl
} // namespace planner
} // namespace aikido

#include "aikido/planner/ompl/detail/OMPLConfigurationToConfigurationPlanner-impl.hpp"

#endif // AIKIDO_PLANNER_SNAPCONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
