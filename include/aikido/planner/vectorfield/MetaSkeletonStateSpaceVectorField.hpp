#ifndef AIKIDO_PLANNER_VECTORFIELD_METASKELETONSTATESPACEVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_METASKELETONSTATESPACEVECTORFIELD_HPP_

#include <aikido/planner/vectorfield/VectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerStatus.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// This class defines a vector field in a configuration space.
///
/// Any vector field should inherit this class to implememnt functions
/// for calculating joint velocities giving joint positions and time,
/// and checking planning status.
class MetaSkeletonStateSpaceVectorField : public VectorField
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace MetaSkeleton state space.
  /// \param[in] bn Body node of end-effector.
  MetaSkeletonStateSpaceVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace);

  // Documentation inherited.
  virtual bool evaluateVelocity(
      const aikido::statespace::StateSpace::State* state,
      Eigen::VectorXd& qd) const = 0;

  // Documentation inherited.
  virtual VectorFieldPlannerStatus evaluateStatus(
      const aikido::statespace::StateSpace::State* state) const = 0;

  // Documentation inherited.
  aikido::constraint::TestablePtr getBoundTestable() const override;

  /// Return meta skeleton state space.
  aikido::statespace::dart::MetaSkeletonStateSpacePtr
  getMetaSkeletonStateSpace();

  /// Return const meta skeleton state space.
  aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
  getMetaSkeletonStateSpace() const;

  /// Returns meta skeleton.
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton();

  /// Returns const meta skeleton.
  dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const;

protected:
  /// Meta skeleton state space.
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// Meta Skeleton
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

using MetaSkeletonStateSpaceVectorFieldPtr
    = std::shared_ptr<MetaSkeletonStateSpaceVectorField>;

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_METASKELETONSTATESPACEVECTORFIELD_HPP_
