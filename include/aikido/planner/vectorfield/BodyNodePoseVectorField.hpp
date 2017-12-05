#ifndef AIKIDO_PLANNER_VECTORFIELD_BODYNODEPOSEVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_BODYNODEPOSEVECTORFIELD_HPP_

#include <dart/dynamics/BodyNode.hpp>
#include <aikido/planner/vectorfield/MetaSkeletonStateSpaceVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// This class defines a vector field in a configuration space.
///
/// Any vector field should inherit this class to implememnt functions
/// for calculating joint velocities giving joint positions and time,
/// and checking planning status.
class BodyNodePoseVectorField : public MetaSkeletonStateSpaceVectorField
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace MetaSkeleton state space.
  /// \param[in] bn Body node of end-effector.
  BodyNodePoseVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bodyNode);

  // Documentation inherited.
  virtual bool evaluateVelocity(
      const aikido::statespace::StateSpace::State* state,
      Eigen::VectorXd& qd) const override = 0;

  // Documentation inherited.
  virtual VectorFieldPlannerStatus evaluateStatus(
      const aikido::statespace::StateSpace::State* state) const override = 0;

  /// Returns body node of end-effector.
  dart::dynamics::BodyNodePtr getBodyNode();

  /// Returns const body node of end-effector.
  dart::dynamics::ConstBodyNodePtr getBodyNode() const;

protected:
  /// BodyNode
  dart::dynamics::BodyNodePtr mBodyNode;
};

using BodyNodePoseVectorFieldPtr = std::shared_ptr<BodyNodePoseVectorField>;

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_BODYNODEPOSEVECTORFIELD_HPP_
