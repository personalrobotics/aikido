#include <cassert>
#include <sstream>
#include <dart/common/Console.hpp>
#include <dart/common/StlHelpers.hpp>
#include <aikido/statespace/dart/JointStateSpaceHelpers.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

using ::dart::dynamics::MetaSkeleton;
using ::dart::dynamics::INVALID_INDEX;

namespace aikido {
namespace statespace {
namespace dart {

using JointStateSpacePtr = std::shared_ptr<JointStateSpace>;

namespace {

//==============================================================================
template <class Input, class Output>
std::vector<Output> convertVectorType(const std::vector<Input>& _input)
{
  std::vector<Output> output;
  output.reserve(_input.size());

  for (const auto& x : _input)
    output.emplace_back(x);

  return std::move(output);
}

//==============================================================================
template <class T>
T* isJointOfType(::dart::dynamics::Joint* _joint)
{
  // It's safe to do a pointer comparison here, since getType is guaranteed to
  // return the same reference to the corresponding getStaticType method.
  if (&_joint->getType() == &T::getStaticType())
    return static_cast<T*>(_joint);
  else
    return nullptr;
}

//==============================================================================
std::vector<std::shared_ptr<JointStateSpace>> createStateSpace(
    MetaSkeleton& _metaskeleton)
{
  std::vector<std::shared_ptr<JointStateSpace>> spaces;
  spaces.reserve(_metaskeleton.getNumJoints());

  for (std::size_t ijoint = 0; ijoint < _metaskeleton.getNumJoints(); ++ijoint)
  {
    const auto joint = _metaskeleton.getJoint(ijoint);

    // Verify that the joint is not missing any DOFs. This could alter the
    // topology of the space that we create.
    for (std::size_t idof = 0; idof < joint->getNumDofs(); ++idof)
    {
      const auto dof = joint->getDof(idof);
      if (_metaskeleton.getIndexOf(dof, false) == INVALID_INDEX)
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
    ::dart::dynamics::MetaSkeleton* _metaskeleton)
  : mName(_metaskeleton->getName())
  , mNumJoints(_metaskeleton->getNumJoints())
  , mPositionLowerLimits(_metaskeleton->getPositionLowerLimits())
  , mPositionUpperLimits(_metaskeleton->getPositionUpperLimits())
  , mVelocityLowerLimits(_metaskeleton->getVelocityLowerLimits())
  , mVelocityUpperLimits(_metaskeleton->getVelocityUpperLimits())
{
  mDofNames.resize(_metaskeleton->getNumDofs());

  for (std::size_t ijoint = 0; ijoint < _metaskeleton->getNumJoints(); ++ijoint)
  {
    const auto joint = _metaskeleton->getJoint(ijoint);

    // FIXME: This duplicates some of the logic in createStateSpace above.
    for (std::size_t idof = 0; idof < joint->getNumDofs(); ++idof)
    {
      const auto dof = joint->getDof(idof);
      const auto dofName = dof->getName();
      const auto dofIndex = _metaskeleton->getIndexOf(dof, false);
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
std::vector<std::string> MetaSkeletonStateSpace::Properties::getDofNames() const
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
const Eigen::VectorXd
MetaSkeletonStateSpace::Properties::getPositionLowerLimits() const
{
  return mPositionLowerLimits;
}

//==============================================================================
const Eigen::VectorXd
MetaSkeletonStateSpace::Properties::getPositionUpperLimits() const
{
  return mPositionUpperLimits;
}

//==============================================================================
const Eigen::VectorXd
MetaSkeletonStateSpace::Properties::getVelocityLowerLimits() const
{
  return mVelocityLowerLimits;
}

//==============================================================================
const Eigen::VectorXd
MetaSkeletonStateSpace::Properties::getVelocityUpperLimits() const
{
  return mVelocityUpperLimits;
}

//==============================================================================
MetaSkeletonStateSpace::MetaSkeletonStateSpace(MetaSkeleton* _metaskeleton)
  : CartesianProduct(
        convertVectorType<JointStateSpacePtr, StateSpacePtr>(
            createStateSpace(*_metaskeleton)))
  , mProperties(MetaSkeletonStateSpace::Properties(_metaskeleton))
{
  // Do nothing.
}

//==============================================================================
const MetaSkeletonStateSpace::Properties MetaSkeletonStateSpace::getProperties()
    const
{
  return mProperties;
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
    ::dart::dynamics::MetaSkeleton* _metaskeleton, const State* _state)
{
  Eigen::VectorXd positions;
  convertStateToPositions(_state, positions);
  _metaskeleton->setPositions(positions);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
