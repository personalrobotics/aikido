#ifndef AIKIDO_PLANNER_VECTORFIELD_CONFIGURATIONSPACEVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_CONFIGURATIONSPACEVECTORFIELD_HPP_

#include <aikido/planner/vectorfield/VectorFieldPlannerStatus.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// This class defines a vector field in a configuration space
///
/// Any vector field should inherit this class to implememnt functions
/// for calculating joint velocities giving joint positions and time,
/// and checking planning status.
class ConfigurationSpaceVectorField
{
public:
  /// Constructor
  ///
  /// \param[in] _stateSpace MetaSkeleton state space
  /// \param[in] _bn Body node of end-effector
  ConfigurationSpaceVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
      dart::dynamics::BodyNodePtr _bodyNode);

  /// Vectorfield callback function
  ///
  /// \param[out] _qd Joint velocities
  /// \return Whether joint velocities are successfully computed
  virtual bool getJointVelocities(Eigen::VectorXd& _qd) const = 0;

  /// Vectorfield planning status callback function
  ///
  /// \return Status of planning
  virtual VectorFieldPlannerStatus checkPlanningStatus() const = 0;

  /// Meta skeleton state space
  aikido::statespace::dart::MetaSkeletonStateSpacePtr
  getMetaSkeletonStateSpace() const;

  /// Meta skeleton
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton() const;

  /// Body node of end-effector
  dart::dynamics::BodyNodePtr getBodyNode() const;

protected:
  /// State space of vector field
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  /// Meta Skeleton of state space
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  /// BodyNode of end-effector
  dart::dynamics::BodyNodePtr mBodyNode;
};

using ConfigurationSpaceVectorFieldPtr
    = std::shared_ptr<ConfigurationSpaceVectorField>;

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_CONFIGURATIONSPACEVECTORFIELD_HPP_
