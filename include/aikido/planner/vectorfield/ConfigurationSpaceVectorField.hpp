#ifndef AIKIDO_PLANNER_VECTORFIELD_CONFIGURATIONSPACEVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_CONFIGURATIONSPACEVECTORFIELD_HPP_

#include <aikido/planner/vectorfield/VectorFieldPlannerStatus.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// This class defines a vector field in a configuration space
///
/// Any vector filed should inherit this class to implememnt functions
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
      dart::dynamics::BodyNodePtr _bodyNode)
    : mStateSpace(_stateSpace)
    , mMetaSkeleton(_stateSpace->getMetaSkeleton())
    , mBodyNode(_bodyNode)
  {
  }

  /// Vectorfield callback function
  ///
  /// \param[in] _q Position in configuration space
  /// \param[in] _t Current time being planned
  /// \param[out] _qd Joint velocities
  /// \return Whether joint velocities are successfully computed
  virtual bool getJointVelocities(
      const Eigen::VectorXd& _q, double _t, Eigen::VectorXd& _qd)
      = 0;

  /// Vectorfield planning status callback function
  ///
  /// \param[in] _q Position in configuration space
  /// \param[in] _t Current time being planned
  /// \return Status of planning
  virtual VectorFieldPlannerStatus checkPlanningStatus(
      const Eigen::VectorXd& _q, double _t)
      = 0;

  /// Meta skeleton spate space
  virtual aikido::statespace::dart::MetaSkeletonStateSpacePtr
  getMetaSkeletonStateSpace()
  {
    return mStateSpace;
  }

  /// Meta skeleton
  virtual dart::dynamics::MetaSkeletonPtr getMetaSkeleton()
  {
    return mMetaSkeleton;
  }

  /// Body node of end-effector
  virtual dart::dynamics::BodyNodePtr getBodyNode()
  {
    return mBodyNode;
  }

protected:
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  dart::dynamics::BodyNodePtr mBodyNode;
};

using ConfigurationSpaceVectorFieldPtr
    = std::shared_ptr<ConfigurationSpaceVectorField>;

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_CONFIGURATIONSPACEVECTORFIELD_HPP_
