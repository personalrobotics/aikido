#include <aikido/statespace/dart/JointStateSpace.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
JointStateSpace::Properties::Properties(::dart::dynamics::Joint* _joint)
  : mName(_joint->getName()), mType(_joint->getType())
{
  const std::size_t numDofs = _joint->getNumDofs();

  mDofNames.resize(numDofs);

  mPositionLowerLimits.resize(numDofs);
  mPositionUpperLimits.resize(numDofs);
  mPositionHasLimits.resize(numDofs);

  mVelocityLowerLimits.resize(numDofs);
  mVelocityUpperLimits.resize(numDofs);

  for (std::size_t index = 0; index < numDofs; ++index)
  {
    mDofNames[index] = _joint->getDofName(index);

    mPositionLowerLimits[index] = _joint->getPositionLowerLimit(index);
    mPositionUpperLimits[index] = _joint->getPositionUpperLimit(index);
    mPositionHasLimits[index] = _joint->hasPositionLimit(index);

    mVelocityLowerLimits[index] = _joint->getVelocityLowerLimit(index);
    mVelocityUpperLimits[index] = _joint->getVelocityUpperLimit(index);
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
JointStateSpace::JointStateSpace(::dart::dynamics::Joint* _joint)
  : mProperties(JointStateSpace::Properties(_joint))
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
    const ::dart::dynamics::Joint* _joint, StateSpace::State* _state) const
{
  convertPositionsToState(_joint->getPositions(), _state);
}

//==============================================================================
void JointStateSpace::setState(
    ::dart::dynamics::Joint* _joint, const StateSpace::State* _state) const
{
  Eigen::VectorXd positions;
  convertStateToPositions(_state, positions);
  _joint->setPositions(positions);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
