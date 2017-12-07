#include <aikido/statespace/dart/JointStateSpace.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
JointStateSpace::Properties::Properties(::dart::dynamics::Joint* _joint)
  : mName(_joint->getName())
  , mType(_joint->getType())
  , mNumDofs(_joint->getNumDofs())
{
  mPositionLowerLimits.resize(mNumDofs);
  mPositionUpperLimits.resize(mNumDofs);
  mPositionHasLimits.resize(mNumDofs);

  for (std::size_t index = 0; index < mNumDofs; ++index)
  {
    mPositionLowerLimits[index] = _joint->getPositionLowerLimit(index);
    mPositionUpperLimits[index] = _joint->getPositionUpperLimit(index);
    mPositionHasLimits[index] = _joint->hasPositionLimit(index);
  }
}

//==============================================================================
const std::string JointStateSpace::Properties::getName() const
{
  return mName;
}

//==============================================================================
const std::string JointStateSpace::Properties::getType() const
{
  return mType;
}

//==============================================================================
std::size_t JointStateSpace::Properties::getNumDofs() const
{
  return mNumDofs;
}

//==============================================================================
bool JointStateSpace::Properties::hasPositionLimit(std::size_t index) const
{
  if (index >= mNumDofs)
  {
    std::stringstream ss;
    ss << "[JointStateSpace::Properties::hasPositionLimit] Index " << index
       << " is out of bounds for Joint with " << mNumDofs << " DOFs.";
    throw std::out_of_range(ss.str());
  }

  return mPositionHasLimits[index];
}

//==============================================================================
bool JointStateSpace::Properties::isLimited() const
{
  return mPositionHasLimits.array().any();
}

//==============================================================================
const Eigen::VectorXd JointStateSpace::Properties::getPositionLowerLimits()
    const
{
  return mPositionLowerLimits;
}

//==============================================================================
const Eigen::VectorXd JointStateSpace::Properties::getPositionUpperLimits()
    const
{
  return mPositionUpperLimits;
}

//==============================================================================
JointStateSpace::JointStateSpace(::dart::dynamics::Joint* _joint)
  : mProperties(JointStateSpace::Properties(_joint))
{
  // Do nothing.
}

//==============================================================================
const JointStateSpace::Properties JointStateSpace::getProperties() const
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
