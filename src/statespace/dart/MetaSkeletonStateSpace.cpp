#include <cassert>
#include <sstream>
#include <dart/common/Console.hpp>
#include <dart/common/StlHelpers.hpp>
#include <aikido/statespace/dart/JointStateSpaceHelpers.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

using ::dart::dynamics::MetaSkeleton;
using ::dart::dynamics::MetaSkeletonPtr;
using ::dart::dynamics::BallJoint;
using ::dart::dynamics::FreeJoint;
using ::dart::dynamics::EulerJoint;
using ::dart::dynamics::PlanarJoint;
using ::dart::dynamics::PrismaticJoint;
using ::dart::dynamics::RevoluteJoint;
using ::dart::dynamics::Joint;
using ::dart::dynamics::ScrewJoint;
using ::dart::dynamics::TranslationalJoint;
using ::dart::dynamics::WeldJoint;
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
  // return the same reference to the corresponding getTypeStatic method.
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

  for (size_t ijoint = 0; ijoint < _metaskeleton.getNumJoints(); ++ijoint)
  {
    const auto joint = _metaskeleton.getJoint(ijoint);

    // Verify that the joint is not missing any DOFs. This could alter the
    // topology of the space that we create.
    for (size_t idof = 0; idof < joint->getNumDofs(); ++idof)
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
MetaSkeletonStateSpace::MetaSkeletonStateSpace(MetaSkeletonPtr _metaskeleton)
  : CartesianProduct(
        convertVectorType<JointStateSpacePtr, StateSpacePtr>(
            createStateSpace(*_metaskeleton)))
  , mMetaSkeleton(std::move(_metaskeleton))
{
}

//==============================================================================
MetaSkeletonPtr MetaSkeletonStateSpace::getMetaSkeleton() const
{
  return mMetaSkeleton;
}

//==============================================================================
void MetaSkeletonStateSpace::convertPositionsToState(
    const Eigen::VectorXd& _positions, State* _state) const
{
  if (static_cast<size_t>(_positions.size()) != mMetaSkeleton->getNumDofs())
    throw std::invalid_argument("Incorrect number of positions.");

  for (size_t isubspace = 0; isubspace < getNumSubspaces(); ++isubspace)
  {
    const auto subspace = getSubspace<JointStateSpace>(isubspace);
    const auto joint = subspace->getJoint();

    // TODO: Find a more efficient way to do this mapping.
    const auto numJointDofs = joint->getNumDofs();
    Eigen::VectorXd jointPositions(numJointDofs);

    for (size_t idof = 0; idof < numJointDofs; ++idof)
    {
      const auto dof = joint->getDof(idof);
      const auto dofIndex = mMetaSkeleton->getIndexOf(dof, false);
      if (dofIndex == INVALID_INDEX)
        throw std::logic_error(
            "DegreeOfFreedom is not in MetaSkeleton. This should never "
            "happen.");

      jointPositions[idof] = _positions[dofIndex];
    }

    const auto substate = getSubState<>(_state, isubspace);
    subspace->convertPositionsToState(jointPositions, substate);
  }
}

//==============================================================================
void MetaSkeletonStateSpace::convertStateToPositions(
    const State* _state, Eigen::VectorXd& _positions) const
{
  _positions.resize(mMetaSkeleton->getNumDofs());

  for (size_t isubspace = 0; isubspace < getNumSubspaces(); ++isubspace)
  {
    const auto subspace = getSubspace<JointStateSpace>(isubspace);
    const auto joint = subspace->getJoint();
    const auto substate = getSubState<>(_state, isubspace);

    Eigen::VectorXd jointPositions;
    subspace->convertStateToPositions(substate, jointPositions);

    // TODO: Find a more efficient way to do this mapping.
    for (size_t idof = 0; idof < static_cast<size_t>(jointPositions.size());
         ++idof)
    {
      const auto dof = joint->getDof(idof);
      const auto dofIndex = mMetaSkeleton->getIndexOf(dof, false);
      if (dofIndex == INVALID_INDEX)
        throw std::logic_error(
            "DegreeOfFreedom is not in MetaSkeleton. This should never "
            "happen.");

      _positions[dofIndex] = jointPositions[idof];
    }
  }
}

//==============================================================================
void MetaSkeletonStateSpace::getState(State* _state) const
{
  convertPositionsToState(mMetaSkeleton->getPositions(), _state);
}

//==============================================================================
auto MetaSkeletonStateSpace::getScopedStateFromMetaSkeleton() const
    -> ScopedState
{
  auto scopedState = createState();
  getState(scopedState.getState());

  return scopedState;
}

//==============================================================================
void MetaSkeletonStateSpace::setState(const State* _state)
{
  Eigen::VectorXd positions;
  convertStateToPositions(_state, positions);
  mMetaSkeleton->setPositions(positions);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
