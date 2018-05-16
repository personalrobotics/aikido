#include "aikido/planner/ConfigurationToEndEffectorPose.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToEndEffectorPose::ConfigurationToEndEffectorPose(
    statespace::ConstStateSpacePtr stateSpace,
    dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const statespace::StateSpace::State* startState,
    const Eigen::Isometry3d& goalPose,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace), std::move(constraint))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mStartState(stateSpace->cloneState(startState))
  , mGoalPose(goalPose)
{
  // Do nothing
}

//==============================================================================
const std::string& ConfigurationToEndEffectorPose::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ConfigurationToEndEffectorPose::getStaticType()
{
  static std::string name("ConfigurationToEndEffectorPose");
  return name;
}

//==============================================================================
dart::dynamics::ConstBodyNodePtr
ConfigurationToEndEffectorPose::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode;
}

//==============================================================================
const statespace::StateSpace::State*
ConfigurationToEndEffectorPose::getStartState() const
{
  return mStartState;
}

//==============================================================================
const Eigen::Isometry3d& ConfigurationToEndEffectorPose::getGoalPose() const
{
  return mGoalPose;
}

} // namespace planner
} // namespace aikido
