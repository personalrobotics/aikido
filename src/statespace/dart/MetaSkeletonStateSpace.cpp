#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

#include <cassert>
#include <sstream>

#include <dart/common/Console.hpp>

#include "aikido/common/memory.hpp"
#include "aikido/statespace/dart/JointStateSpaceHelpers.hpp"

namespace aikido {
namespace statespace {
namespace dart {

namespace {

using ::dart::dynamics::INVALID_INDEX;
using ::dart::dynamics::MetaSkeleton;

using JointStateSpacePtr = std::shared_ptr<JointStateSpace>;
using ConstJointStateSpacePtr = std::shared_ptr<const JointStateSpace>;

//==============================================================================
template <class Input, class Output>
std::vector<Output> convertVectorType(const std::vector<Input>& input)
{
  std::vector<Output> output;
  output.reserve(input.size());

  for (const auto& x : input)
    output.emplace_back(x);

  return output;
}

//==============================================================================
template <class T>
T* isJointOfType(const ::dart::dynamics::Joint* joint)
{
  // It's safe to do a pointer comparison here, since getType is guaranteed to
  // return the same reference to the corresponding getStaticType method.
  if (&joint->getType() == &T::getStaticType())
    return static_cast<T*>(joint);
  else
    return nullptr;
}

//==============================================================================
std::vector<std::shared_ptr<const JointStateSpace>> createStateSpace(
    const MetaSkeleton& metaskeleton)
{
  std::vector<std::shared_ptr<const JointStateSpace>> spaces;
  spaces.reserve(metaskeleton.getNumJoints());

  for (std::size_t ijoint = 0; ijoint < metaskeleton.getNumJoints(); ++ijoint)
  {
    const auto joint = metaskeleton.getJoint(ijoint);

    // Verify that the joint is not missing any DOFs. This could alter the
    // topology of the space that we create.
    for (std::size_t idof = 0; idof < joint->getNumDofs(); ++idof)
    {
      const auto dof = joint->getDof(idof);
      if (metaskeleton.getIndexOf(dof, false) == INVALID_INDEX)
      {
        std::stringstream msg;
        msg << "MetaSkeleton is missing DegreeOfFreedom '" << dof->getName()
            << "' (index: " << idof << ") of Joint '" << joint->getName()
            << "'.";
        throw std::runtime_error(msg.str());
      }
    }

    spaces.emplace_back(createJointStateSpace(joint).release());
  }

  return spaces;
}

} // namespace

//==============================================================================
MetaSkeletonStateSpace::Properties::Properties(
    const ::dart::dynamics::MetaSkeleton* metaskeleton)
  : mName(metaskeleton->getName())
  , mNumJoints(metaskeleton->getNumJoints())
  , mDofNames(metaskeleton->getNumDofs())
  , mPositionLowerLimits(metaskeleton->getPositionLowerLimits())
  , mPositionUpperLimits(metaskeleton->getPositionUpperLimits())
  , mVelocityLowerLimits(metaskeleton->getVelocityLowerLimits())
  , mVelocityUpperLimits(metaskeleton->getVelocityUpperLimits())
{
  // TODO: Acquire the metaskeleton's mutex once DART supports it.

  for (std::size_t ijoint = 0; ijoint < metaskeleton->getNumJoints(); ++ijoint)
  {
    const auto joint = metaskeleton->getJoint(ijoint);

    // FIXME: This duplicates some of the logic in createStateSpace above.
    for (std::size_t idof = 0; idof < joint->getNumDofs(); ++idof)
    {
      const auto dof = joint->getDof(idof);
      const auto& dofName = dof->getName();
      const auto dofIndex = metaskeleton->getIndexOf(dof, false);
      if (dofIndex == INVALID_INDEX)
      {
        std::stringstream msg;
        msg << "MetaSkeleton is missing DegreeOfFreedom '" << dof->getName()
            << "' (index: " << idof << ") of Joint '" << joint->getName()
            << "'. This should never happen.";
        throw std::runtime_error(msg.str());
      }

      mDofNames[dofIndex] = dofName;
      mIndexMap[std::make_pair(ijoint, idof)] = dofIndex;
    }
  }
}

//==============================================================================
const std::string& MetaSkeletonStateSpace::Properties::getName() const
{
  return mName;
}

//==============================================================================
std::size_t MetaSkeletonStateSpace::Properties::getNumJoints() const
{
  return mNumJoints;
}

//==============================================================================
std::size_t MetaSkeletonStateSpace::Properties::getNumDofs() const
{
  return mDofNames.size();
}

//==============================================================================
const std::vector<std::string>&
MetaSkeletonStateSpace::Properties::getDofNames() const
{
  return mDofNames;
}

//==============================================================================
std::size_t MetaSkeletonStateSpace::Properties::getDofIndex(
    std::size_t ijoint, std::size_t ijointdof) const
{
  return mIndexMap.at(std::make_pair(ijoint, ijointdof));
}

//==============================================================================
std::size_t MetaSkeletonStateSpace::Properties::getDofIndex(
    const std::string& dofName) const
{
  std::vector<std::size_t> indices;
  indices.reserve(mDofNames.size());

  for (std::size_t idof = 0; idof < mDofNames.size(); ++idof)
  {
    if (mDofNames[idof] == dofName)
      indices.push_back(idof);
  }

  if (indices.size() == 0)
  {
    std::stringstream message;
    message << "DegreeOfFreedom '" << dofName
            << "' does not exist in MetaSkeleton.";
    throw std::invalid_argument{message.str()};
  }
  else if (indices.size() > 1)
  {
    std::stringstream message;
    message << "Multiple (" << indices.size() << ") DOFs have the same name '"
            << dofName << "'.";
    throw std::invalid_argument{message.str()};
  }

  assert(indices[0] != INVALID_INDEX);
  return indices[0];
}

//==============================================================================
const Eigen::VectorXd&
MetaSkeletonStateSpace::Properties::getPositionLowerLimits() const
{
  return mPositionLowerLimits;
}

//==============================================================================
const Eigen::VectorXd&
MetaSkeletonStateSpace::Properties::getPositionUpperLimits() const
{
  return mPositionUpperLimits;
}

//==============================================================================
const Eigen::VectorXd&
MetaSkeletonStateSpace::Properties::getVelocityLowerLimits() const
{
  return mVelocityLowerLimits;
}

//==============================================================================
const Eigen::VectorXd&
MetaSkeletonStateSpace::Properties::getVelocityUpperLimits() const
{
  return mVelocityUpperLimits;
}

//==============================================================================
bool MetaSkeletonStateSpace::Properties::operator==(
    const Properties& otherProperties) const
{
  if (mName != otherProperties.mName)
    return false;

  if (mNumJoints != otherProperties.mNumJoints)
    return false;

  if (mDofNames != otherProperties.mDofNames)
    return false;

  if (mIndexMap != otherProperties.mIndexMap)
    return false;

  for (std::size_t i = 0; i < mNumJoints; ++i)
  {
    if (mPositionLowerLimits[i] != otherProperties.mPositionLowerLimits[i])
      return false;
    if (mPositionUpperLimits[i] != otherProperties.mPositionUpperLimits[i])
      return false;
    if (mVelocityLowerLimits[i] != otherProperties.mVelocityLowerLimits[i])
      return false;
    if (mVelocityUpperLimits[i] != otherProperties.mVelocityUpperLimits[i])
      return false;
  }

  return true;
}

//==============================================================================
bool MetaSkeletonStateSpace::Properties::operator!=(
    const Properties& otherProperties) const
{
  return !(*this == otherProperties);
}

//==============================================================================
MetaSkeletonStateSpace::MetaSkeletonStateSpace(const MetaSkeleton* metaskeleton)
  : CartesianProduct(
      convertVectorType<ConstJointStateSpacePtr, ConstStateSpacePtr>(
          createStateSpace(*metaskeleton)))
  , mProperties(MetaSkeletonStateSpace::Properties(metaskeleton))
{
  // Do nothing.
}

//==============================================================================
const MetaSkeletonStateSpace::Properties&
MetaSkeletonStateSpace::getProperties() const
{
  return mProperties;
}

//==============================================================================
bool MetaSkeletonStateSpace::isCompatible(
    const ::dart::dynamics::MetaSkeleton* metaskeleton) const
{
  const MetaSkeletonStateSpace::Properties otherProperties(metaskeleton);
  return mProperties == otherProperties;
}

//==============================================================================
void MetaSkeletonStateSpace::checkCompatibility(
    const ::dart::dynamics::MetaSkeleton* metaskeleton) const
{
  if (isCompatible(metaskeleton))
    return;

  std::stringstream ss;
  ss << "MetaSkeleton '" << metaskeleton->getName()
     << "' does not match this MetaSkeletonStateSpace's MetaSkeleton '"
     << mProperties.getName() << "'.";
  throw std::invalid_argument(ss.str());
}

//==============================================================================
void MetaSkeletonStateSpace::checkIfContained(
    const ::dart::dynamics::Skeleton* skeleton) const
{
  // TODO: Name-uniqueness is allowed only within the same skeleton,
  // so we should check for skeleton-equality.
  auto dofNames = mProperties.getDofNames();
  for (const auto& name : dofNames)
  {
    if (!skeleton->getDof(name))
    {
      std::stringstream ss;
      ss << "DegreeOfFreedom[" << name << "] does not exist in mSkeleton.";
      throw std::invalid_argument(ss.str());
    }
  }
}

//==============================================================================
void MetaSkeletonStateSpace::convertPositionsToState(
    const Eigen::VectorXd& _positions, State* _state) const
{
  if (static_cast<std::size_t>(_positions.size()) != mProperties.getNumDofs())
    throw std::invalid_argument("Incorrect number of positions.");

  for (std::size_t isubspace = 0; isubspace < getNumSubspaces(); ++isubspace)
  {
    const auto subspace = getSubspace<JointStateSpace>(isubspace);
    const auto numJointDofs = subspace->getProperties().getNumDofs();

    Eigen::VectorXd jointPositions(numJointDofs);
    for (std::size_t idof = 0; idof < numJointDofs; ++idof)
      jointPositions[idof]
          = _positions[mProperties.getDofIndex(isubspace, idof)];

    const auto substate = getSubState<>(_state, isubspace);
    subspace->convertPositionsToState(jointPositions, substate);
  }
}

//==============================================================================
void MetaSkeletonStateSpace::convertStateToPositions(
    const State* _state, Eigen::VectorXd& _positions) const
{
  _positions.resize(mProperties.getNumDofs());

  for (std::size_t isubspace = 0; isubspace < getNumSubspaces(); ++isubspace)
  {
    const auto subspace = getSubspace<JointStateSpace>(isubspace);
    const auto numJointDofs = subspace->getProperties().getNumDofs();
    const auto substate = getSubState<>(_state, isubspace);

    Eigen::VectorXd jointPositions(numJointDofs);
    subspace->convertStateToPositions(substate, jointPositions);

    for (std::size_t idof = 0; idof < numJointDofs; ++idof)
      _positions[mProperties.getDofIndex(isubspace, idof)]
          = jointPositions[idof];
  }
}

//==============================================================================
void MetaSkeletonStateSpace::getState(
    const ::dart::dynamics::MetaSkeleton* _metaskeleton, State* _state) const
{
  convertPositionsToState(_metaskeleton->getPositions(), _state);
}

//==============================================================================
auto MetaSkeletonStateSpace::getScopedStateFromMetaSkeleton(
    const ::dart::dynamics::MetaSkeleton* _metaskeleton) const -> ScopedState
{
  auto scopedState = createState();
  getState(_metaskeleton, scopedState.getState());

  return scopedState;
}

//==============================================================================
void MetaSkeletonStateSpace::setState(
    ::dart::dynamics::MetaSkeleton* _metaskeleton, const State* _state) const
{
  Eigen::VectorXd positions;
  convertStateToPositions(_state, positions);
  _metaskeleton->setPositions(positions);
}

//==============================================================================
::dart::dynamics::MetaSkeletonPtr
MetaSkeletonStateSpace::getControlledMetaSkeleton(
    const ::dart::dynamics::SkeletonPtr& skeleton) const
{
  using ::dart::dynamics::DegreeOfFreedom;
  using ::dart::dynamics::Group;

  std::vector<DegreeOfFreedom*> dofs;
  dofs.reserve(mProperties.getNumDofs());

  for (const auto& dofName : mProperties.getDofNames())
  {
    DegreeOfFreedom* dof = skeleton->getDof(dofName);
    if (!dof)
    {
      throw std::invalid_argument(
          "Skeleton has no DegreeOfFreedom named '" + dofName + "'.");
    }
    dofs.emplace_back(dof);
  }

  auto controlledMetaSkeleton
      = Group::create(mProperties.getName(), dofs, false, true);
  if (!controlledMetaSkeleton)
  {
    throw std::runtime_error(
        "Failed creating MetaSkeleton of controlled DOFs.");
  }

  if (controlledMetaSkeleton->getNumDofs() != mProperties.getNumDofs())
  {
    throw std::runtime_error("Only single-DOF joints are supported.");
  }

  return controlledMetaSkeleton;
}

} // namespace dart
} // namespace statespace
} // namespace aikido
