#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDINTEGRATOR_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDINTEGRATOR_HPP_

#include <functional>
#include <dart/common/Timer.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/planner/vectorfield/VectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerExceptions.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerStatus.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

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
  /// \param[in] initialStepSize Initial step size in integation.
  VectorFieldIntegrator(
      const VectorFieldPtr vectorField,
      const aikido::constraint::TestablePtr collisionFreeConstraint,
      double initialStepSize);

  /// Generate a trajectory following the vector field along given time.
  ///
  /// \param[in] integrationTimeInterval Position in configuration space.
  /// \param[in] timelimit Timelimit for integration calculation.
  /// \return A trajectory following the vector field.
  std::unique_ptr<aikido::trajectory::Spline> followVectorField(
      const aikido::statespace::StateSpace::State* startState,
      double timelimit);

  /// Convert a sequence of waypoint and time pairs into a trajectory.
  ///
  /// \param[in] A seqeunce of waypoint and time pairs.
  /// \param[in] stateSpace State space of output trajectory
  /// \return A trajectory
  std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
      const std::vector<Knot>& knots,
      const aikido::statespace::StateSpacePtr stateSpace);

  /// Get initial step size.
  ///
  /// \return Initial step size.
  double getInitialStepSize();

  /// Get resolution of constraint checking.
  ///
  /// \return Resolution of constraint checking.
  double getConstraintCheckResolution();

  /// Set resolution of constraint checking.
  ///
  /// /param[in] Resolution used in constraint checking.
  void setConstraintCheckResolution(double resolution);

protected:
  /// Evaluate whether a trajectory is eligible
  ///
  ///
  virtual bool evaluateTrajectory(
      const aikido::trajectory::Trajectory* trajectory,
      const aikido::constraint::TestablePtr collisionFreeConstraint,
      double evalStepSize)
      = 0;

  virtual bool convertStateToPositions(
      const aikido::statespace::StateSpace::State* state,
      Eigen::VectorXd& positions)
      = 0;

  virtual bool convertPositionsToState(
      const Eigen::VectorXd& positions,
      aikido::statespace::StateSpace::State* state)
      = 0;

  /// Vectorfield callback function that returns joint velocities for
  /// integration.
  ///
  /// \param[in] q Position in configuration space.
  /// \param[out] qd Joint velocities in configuration space.
  /// \param[in] t Current time being planned.
  virtual void step(const Eigen::VectorXd& q, Eigen::VectorXd& qd, double t);

  /// Check status after every intergration step.
  ///
  /// \param[in] q Position in configuration space.
  /// \param[in] t Current time being planned.
  virtual void check(const Eigen::VectorXd& q, double t);

  VectorFieldPtr mVectorField;
  aikido::constraint::TestablePtr mCollisionFreeConstraint;

  /// Initial step size for adaptive integrator.
  double mInitialStepSize;

  /// Cached index of knots.
  int mCacheIndex;

  /// current index of knots.
  int mIndex;

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
