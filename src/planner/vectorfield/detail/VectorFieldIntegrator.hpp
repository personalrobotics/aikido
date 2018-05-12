#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDINTEGRATOR_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDINTEGRATOR_HPP_

#include <dart/common/Timer.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/planner/vectorfield/VectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerStatus.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {
namespace detail {

struct Knot
{
  /// Timestamp.
  double mT;

  /// Positions.
  Eigen::VectorXd mPositions;
};

/// Convert a sequence of waypoint and time pairs into a trajectory.
///
/// \param[in] A seqeunce of waypoint and time pairs.
/// \param[in] stateSpace State space of output trajectory.
/// \return A trajectory.
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const std::vector<Knot>& knots,
    aikido::statespace::ConstStateSpacePtr stateSpace);



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
  /// \param[in] timelimit Timelimit for integration calculation.
  /// \param[in] checkConstraintResolution Resolution used in checking
  /// constraint satisfaction in generated trajectory.
  VectorFieldIntegrator(
      const VectorField* vectorField,
      const aikido::constraint::Testable* constraint,
      double timelimit,
      double checkConstraintResolution);

  /// Called before doing integration.
  ///
  void start();

  /// Get cache index.
  ///
  /// \return Cache index.
  int getCacheIndex();

  /// Get a list of knots stored in the integration
  ///
  /// \return a list of knots.
  std::vector<Knot>& getKnots();

  /// Get last evaluation time.
  ///
  /// \return last evaluation time.
  double getLastEvaluationTime();

  /// Vectorfield callback function that returns joint velocities for
  /// integration.
  ///
  /// \param[in] q Position in configuration space.
  /// \param[out] qd Joint velocities in configuration space.
  /// \param[in] t Current time being planned.
  void step(const Eigen::VectorXd& q, Eigen::VectorXd& qd, double t);

  /// Check status after every integration step.
  ///
  /// \param[in] q Position in configuration space.
  /// \param[in] t Current time being planned.
  void check(const Eigen::VectorXd& q, double t);

protected:
  /// Vector field
  const VectorField* mVectorField;

  /// Constraint to be satisfied
  const aikido::constraint::Testable* mConstraint;

  /// Cached index of knots.
  int mCacheIndex;

  /// Dimension of state space.
  std::size_t mDimension;

  /// Timer for timelimit.
  dart::common::Timer mTimer;

  /// Planning timelimit.
  double mTimelimit;

  std::vector<Knot> mKnots;

  /// Resolution used in checking constraint satisfaction.
  double mConstraintCheckResolution;

  /// Current state in integration
  aikido::statespace::StateSpace::State* mState;

  /// Last evaluation time in checking trajectory
  double mLastEvaluationTime;
};

} // namespace detail
} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDINTEGRATOR_HPP_
