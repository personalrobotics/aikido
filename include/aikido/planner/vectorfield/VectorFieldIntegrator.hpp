#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDINTEGRATOR_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDINTEGRATOR_HPP_

#include <dart/common/Timer.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/vectorfield/VectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerStatus.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

struct Knot
{
  /// Timestamp.
  double mT;

  /// Positions.
  Eigen::VectorXd mPositions;
};

/// VectorField Planner generates a trajectory by following a vector field
/// defined in joint space.
/// A planned trajectroy ends either by a predefined termination criterion or a
/// integral time.
///
/// This class defines two callback functions for a integrator.
/// step() provides joint velocities the vector field planner should follow,
/// check() is called after each integration step to check planner status.
class VectorFieldIntegrator
{
public:
  /// Constructor.
  ///
  /// \param[in] vectorField Vector field in configuration space.
  /// \param[in] constraint Constraint to be satisfied.
  VectorFieldIntegrator(
      VectorFieldPtr vectorField, aikido::constraint::TestablePtr constraint);

  /// Generate a trajectory following the vector field along given time.
  ///
  /// \param[in] integrationTimeInterval Position in configuration space.
  /// \param[in] timelimit Timelimit for integration calculation.
  /// \param[in] initialStepSize Initial step size of integator in following
  /// vector field.
  /// \param[in] checkConstraintResolution Resolution used in checking
  /// constraint
  /// satisfaction in generated trajectory.
  /// \param[out] planningResult information about success or failure.
  /// \return A trajectory following the vector field.
  std::unique_ptr<aikido::trajectory::Spline> followVectorField(
      const aikido::statespace::StateSpace::State* startState,
      std::chrono::duration<double> timelimit,
      double initialStepSize,
      double checkConstraintResolution,
      planner::PlanningResult* planningResult);

protected:
  /// Vectorfield callback function that returns joint velocities for
  /// integration.
  ///
  /// \param[in] q Position in configuration space.
  /// \param[out] qd Joint velocities in configuration space.
  /// \param[in] t Current time being planned.
  virtual void step(const Eigen::VectorXd& q, Eigen::VectorXd& qd, double t);

  /// Check status after every integration step.
  ///
  /// \param[in] q Position in configuration space.
  /// \param[in] t Current time being planned.
  virtual void check(const Eigen::VectorXd& q, double t);

  /// Vector field
  VectorFieldPtr mVectorField;

  /// Constraint to be satisfied
  aikido::constraint::TestablePtr mConstraint;

  /// Cached index of knots.
  int mCacheIndex;

  /// Current index of knots.
  int mIndex;

  /// Dimension of state space.
  std::size_t mDimension;

  /// Timer for timelimit.
  dart::common::Timer mTimer;

  /// Planning timelimit.
  double mTimelimit;

  std::vector<Knot> mKnots;

  /// Resolution used in checking constraint satisfaction.
  double mConstraintCheckResolution;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDINTEGRATOR_HPP_
