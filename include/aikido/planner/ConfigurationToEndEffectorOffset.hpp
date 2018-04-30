#ifndef AIKIDO_PLANNER_PLANTOENDEFFECTOROFFSET_HPP_
#define AIKIDO_PLANNER_PLANTOENDEFFECTOROFFSET_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to desired end-effector offset while maintaining
/// the current end-effector orientation.
class ConfigurationToEndEffectorOffset : public Problem
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// offest while maintaining the current orientation.
  /// \param[in] startState Start state.
  /// \param[in] direction Direction of motion [unit vector in the world frame].
  /// \param[in] distance Distance to move, in meters.
  /// \param[in] interpolator Interpolator used to produce the output
  /// trajectory.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToEndEffectorOffset(
      statespace::ConstStateSpacePtr stateSpace,
      dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      const statespace::StateSpace::State* startState,
      const Eigen::Vector3d& direction,
      double distance,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the end-effector BodyNode to be planned to move a desired offest
  /// while maintaining the current orientation.
  dart::dynamics::ConstBodyNodePtr getEndEffectorBodyNode() const;

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Returns the direction of motion specified in the world frame.
  const Eigen::Vector3d& getDirection() const;

  /// Returns the distance in meters to move in the specified direction.
  double getDistance() const;

protected:
  /// End-effector body node.
  const dart::dynamics::ConstBodyNodePtr mEndEffectorBodyNode;

  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Direction of motion.
  const Eigen::Vector3d mDirection;

  /// Distance to move in the direction specified.
  const double mDistance;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANTOENDEFFECTOROFFSET_HPP_
