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
  // TODO: Acquire the joint's mutex once DART supports it.

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
bool JointStateSpace::Properties::operator==(
    const Properties& otherProperties) const
{
  if (mName != otherProperties.mName)
    return false;

  if (mType != otherProperties.mType)
    return false;

  if (mDofNames != otherProperties.mDofNames)
    return false;

  for (std::size_t i = 0; i < mDofNames.size(); ++i)
  {
    if (mPositionLowerLimits[i] != otherProperties.mPositionLowerLimits[i])
      return false;
    if (mPositionUpperLimits[i] != otherProperties.mPositionUpperLimits[i])
      return false;
    if (mPositionHasLimits[i] != otherProperties.mPositionHasLimits[i])
      return false;
    if (mVelocityLowerLimits[i] != otherProperties.mVelocityLowerLimits[i])
      return false;
    if (mVelocityUpperLimits[i] != otherProperties.mVelocityUpperLimits[i])
      return false;
    // if (mVelocityHasLimits[i] != otherProperties.mVelocityHasLimits[i])
    //   return false;
  }

  return true;
}

//==============================================================================
bool JointStateSpace::Properties::operator!=(
    const Properties& otherProperties) const
{
  return !(*this == otherProperties);
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
bool JointStateSpace::isCompatible(const ::dart::dynamics::Joint* joint) const
{
  const JointStateSpace::Properties otherProperties(joint);
  return mProperties == otherProperties;
}

//==============================================================================
void JointStateSpace::checkCompatibility(
    const ::dart::dynamics::Joint* joint) const
{
  if (isCompatible(joint))
    return;

  std::stringstream ss;
  ss << joint->getType() << " '" << joint->getName()
     << "' does not match this JointStateSpace's " << mProperties.getType()
     << " '" << mProperties.getName() << "'.";
  throw std::invalid_argument(ss.str());
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
