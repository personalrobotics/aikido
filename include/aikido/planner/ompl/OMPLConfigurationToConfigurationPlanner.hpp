#ifndef AIKIDO_PLANNER_OMPL_OMPLCONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
#define AIKIDO_PLANNER_OMPL_OMPLCONFIGURATIONTOCONFIGURATIONPLANNER_HPP_

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/geometric/PathSimplifier.h>
#include "aikido/common/RNG.hpp"
#include "aikido/constraint/Projectable.hpp"
#include "aikido/constraint/Sampleable.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/distance/DistanceMetric.hpp"
#include "aikido/planner/ConfigurationToConfiguration.hpp"
#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/ompl/GeometricStateSpace.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace planner {
namespace ompl {

/// Creates an OMPL Planner.
///
/// \tparam PlannerType The OMPL Planner to use.
template <class PlannerType>
class OMPLConfigurationToConfigurationPlanner
    : public aikido::planner::ConfigurationToConfigurationPlanner
{
public:
  // Documentation inherited.
  PlannerPtr clone(common::RNG* rng = nullptr) const override;

  // Documentation inherited.
  bool stopPlanning() override;

  /// Constructor.
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  /// \param[in] rng Random number generator to create the state sampler.
  /// \param[in] interpolator Interpolator used to interpolate between two
  /// states. GeodesicInterpolator is used by default.
  /// \param[in] dmetric A valid distance metric defined on the StateSpace.
  /// Distance metric relevant to the statespace is used by default.
  /// \param[in] sampler A Sampleable to sample states from StateSpace.
  /// \note OMPL planners assume this sampler samples from the statespace
  /// uniformly. Care must be taken when using a non-uniform sampler.
  /// \param[in] boundsConstraint A constraint used to determine whether states
  /// encountered during planning fall within any bounds specified on the
  /// StateSpace. In addition to the validityConstraint, this must also be
  /// satisfied for a state to be considered valid.
  /// \param[in] boundsProjector A Projectable that projects a state back within
  /// valid bounds defined on the StateSpace.
  /// \param[in] maxDistanceBtwValidityChecks The maximum distance (under
  /// dmetric) between validity checking two successive points on a tree
  /// extension or an edge in a graph.
  OMPLConfigurationToConfigurationPlanner(
      statespace::ConstStateSpacePtr stateSpace,
      common::RNG* rng = nullptr,
      statespace::ConstInterpolatorPtr interpolator = nullptr,
      distance::DistanceMetricPtr dmetric = nullptr,
      constraint::SampleablePtr sampler = nullptr,
      constraint::ConstTestablePtr boundsConstraint = nullptr,
      constraint::ProjectablePtr boundsProjector = nullptr,
      double maxDistanceBtwValidityChecks = 0.1);

  /// Plans a trajectory from start state to goal state by using an interpolator
  /// to interpolate between them.
  ///
  /// If successful, the planner returns a trajectory that satisfies the
  /// constraint. If not, it returns a \c nullptr.
  /// The corresponding message is stored in result.
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

protected:
  /// Pointer to the underlying OMPL Planner.
  ::ompl::base::PlannerPtr mPlanner;
};

} // namespace ompl
} // namespace planner
} // namespace aikido

#include "aikido/planner/ompl/detail/OMPLConfigurationToConfigurationPlanner-impl.hpp"

#endif // AIKIDO_PLANNER_OMPL_OMPLCONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
