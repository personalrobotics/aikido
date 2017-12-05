#ifndef AIKIDO_PLANNER_VECTORFIELD_METASKELETONSTATESPACEVECTORFIELDINTEGRATOR_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_METASKELETONSTATESPACEVECTORFIELDINTEGRATOR_HPP_

#include <functional>
#include <dart/common/Timer.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/planner/vectorfield/MetaSkeletonStateSpaceVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldIntegrator.hpp>
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
class MetaSkeletonStateSpaceVectorFieldIntegrator : public VectorFieldIntegrator
{
public:
  /// Constructor.
  ///
  /// \param[in] vectorField Vector field in configuration space.
  /// \param[in] constraint Constraint to be satisfied.
  /// \param[in] initialStepSize Initial step size in integation.
  MetaSkeletonStateSpaceVectorFieldIntegrator(
      const MetaSkeletonStateSpaceVectorFieldPtr vectorField,
      const aikido::constraint::TestablePtr constraint,
      double initialStepSize);

protected:
  // Documentation inherited.
  bool evaluateTrajectory(
      const aikido::trajectory::Trajectory* trajectory,
      const aikido::constraint::TestablePtr collisionFreeConstraint,
      double evalStepSize) override;

  // Documentation inherited.
  bool convertStateToPositions(
      const aikido::statespace::StateSpace::State* state,
      Eigen::VectorXd& positions) override;

  // Documentation inherited.
  bool convertPositionsToState(
      const Eigen::VectorXd& positions,
      aikido::statespace::StateSpace::State* state) override;

  /// Meta Skeleton state space.
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// Meta skeleton.
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_METASKELETONSTATESPACEVECTORFIELDINTEGRATOR_HPP_
