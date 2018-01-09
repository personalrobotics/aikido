#include "aikido/statespace/dart/JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
JointStateSpace::Properties::Properties(const ::dart::dynamics::Joint* joint)
  : mName(joint->getName())
  , mType(joint->getType())
  , mDofNames(joint->getNumDofs())
  , mPositionLowerLimits(joint->getNumDofs())
  , mPositionUpperLimits(joint->getNumDofs())
  , mPositionHasLimits(joint->getNumDofs())
  , mVelocityLowerLimits(joint->getNumDofs())
  , mVelocityUpperLimits(joint->getNumDofs())
{
  for (std::size_t index = 0; index < joint->getNumDofs(); ++index)
  {
    mDofNames[index] = joint->getDofName(index);

    mPositionLowerLimits[index] = joint->getPositionLowerLimit(index);
    mPositionUpperLimits[index] = joint->getPositionUpperLimit(index);
    mPositionHasLimits[index] = joint->hasPositionLimit(index);

    mVelocityLowerLimits[index] = joint->getVelocityLowerLimit(index);
    mVelocityUpperLimits[index] = joint->getVelocityUpperLimit(index);
  }
}

//==============================================================================
const std::string& JointStateSpace::Properties::getName() const
{
  return mName;
}

//==============================================================================
const std::string& JointStateSpace::Properties::getType() const
{
  return mType;
}

//==============================================================================
std::size_t JointStateSpace::Properties::getNumDofs() const
{
  return mDofNames.size();
}

//==============================================================================
const std::vector<std::string>& JointStateSpace::Properties::getDofNames() const
{
  return mDofNames;
}

//==============================================================================
bool JointStateSpace::Properties::hasPositionLimit(std::size_t index) const
{
  if (index >= getNumDofs())
  {
    std::stringstream ss;
    ss << "[JointStateSpace::Properties::hasPositionLimit] Index " << index
       << " is out of bounds for Joint with " << getNumDofs() << " DOFs.";
    throw std::out_of_range(ss.str());
  }

  return mPositionHasLimits[index];
}

//==============================================================================
bool JointStateSpace::Properties::isPositionLimited() const
{
  return mPositionHasLimits.array().any();
}

//==============================================================================
const Eigen::VectorXd& JointStateSpace::Properties::getPositionLowerLimits()
    const
{
  return mPositionLowerLimits;
}

//==============================================================================
const Eigen::VectorXd& JointStateSpace::Properties::getPositionUpperLimits()
    const
{
  return mPositionUpperLimits;
}

//==============================================================================
const Eigen::VectorXd& JointStateSpace::Properties::getVelocityLowerLimits()
    const
{
  return mVelocityLowerLimits;
}

//==============================================================================
const Eigen::VectorXd& JointStateSpace::Properties::getVelocityUpperLimits()
    const
{
  return mVelocityUpperLimits;
}

//==============================================================================
JointStateSpace::JointStateSpace(const ::dart::dynamics::Joint* joint)
  : mProperties(JointStateSpace::Properties(joint))
{
  // Do nothing.
}

//==============================================================================
const JointStateSpace::Properties& JointStateSpace::getProperties() const
{
  return mProperties;
}

//==============================================================================
void JointStateSpace::getState(
    const ::dart::dynamics::Joint* joint, StateSpace::State* state) const
{
  convertPositionsToState(joint->getPositions(), state);
}

//==============================================================================
void JointStateSpace::setState(
    ::dart::dynamics::Joint* joint, const StateSpace::State* state) const
{
  Eigen::VectorXd positions;
  convertStateToPositions(state, positions);
  joint->setPositions(positions);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
