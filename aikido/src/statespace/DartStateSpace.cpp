#include <cassert>
#include <sstream>
#include <aikido/statespace/DartStateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/SO3StateSpace.hpp>
#include <aikido/statespace/SE2StateSpace.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>
#include <dart/common/StlHelpers.h>
#include <dart/common/console.h>

using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::BallJoint;
using dart::dynamics::FreeJoint;
using dart::dynamics::EulerJoint;
using dart::dynamics::PlanarJoint;
using dart::dynamics::PrismaticJoint;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::Joint;
using dart::dynamics::ScrewJoint;
using dart::dynamics::TranslationalJoint;
using dart::dynamics::SingleDofJoint;
using dart::dynamics::WeldJoint;
using dart::dynamics::INVALID_INDEX;
using dart::common::make_unique;

namespace aikido {
namespace statespace {

//=============================================================================
class JointStateSpace
{
public:
  JointStateSpace(
    std::shared_ptr<StateSpace> _space, dart::dynamics::Joint* _joint);

  virtual ~JointStateSpace() = default;

  const std::shared_ptr<StateSpace>& getStateSpace();
  dart::dynamics::Joint* getJoint();

  virtual void getState(StateSpace::State* _state) = 0;
  virtual void setState(const StateSpace::State* _state) = 0;

protected:
  std::shared_ptr<StateSpace> mStateSpace;
  dart::dynamics::Joint* mJoint;
};

namespace {

//=============================================================================
template <class T>
T* isJointOfType(dart::dynamics::Joint* _joint)
{
  // It's safe to do a pointer comparison here, since getType is guaranteed to
  // return the same reference to the corresponding getTypeStatic method.
  if (&_joint->getType() == &T::getStaticType())
    return static_cast<T*>(_joint);
  else
    return nullptr;
}

//=============================================================================
class RealVectorJointStateSpace : public JointStateSpace
{
public:
  RealVectorJointStateSpace(
        std::shared_ptr<RealVectorStateSpace> _space, Joint* _joint)
    : JointStateSpace(_space, _joint)
  {
    assert(_space->getDimension() == _joint->getNumDofs());
  }

  void getState(StateSpace::State* _state) override
  {
    static_cast<RealVectorStateSpace*>(mStateSpace.get())->setValue(
      static_cast<RealVectorStateSpace::State*>(_state),
      mJoint->getPositions());
  }

  void setState(const StateSpace::State* _state) override
  {
    mJoint->setPositions(
      static_cast<RealVectorStateSpace*>(mStateSpace.get())->getValue(
        static_cast<const RealVectorStateSpace::State*>(_state)));
  }
};

//=============================================================================
class SO2JointStateSpace : public JointStateSpace
{
public:
  SO2JointStateSpace(std::shared_ptr<SO2StateSpace> _space, Joint* _joint)
    : JointStateSpace(_space, _joint)
  {
    assert(_joint->getNumDofs() == 1);
  }

  void getState(StateSpace::State* _state) override
  {
    static_cast<SO2StateSpace*>(mStateSpace.get())->setAngle(
      static_cast<SO2StateSpace::State*>(_state),
      mJoint->getPosition(0));
  }

  void setState(const StateSpace::State* _state) override
  {
    mJoint->setPosition(0,
      static_cast<SO2StateSpace*>(mStateSpace.get())->getAngle(
        static_cast<const SO2StateSpace::State*>(_state)));
  }
};

//=============================================================================
class SE3JointStateSpace : public JointStateSpace
{
public:
  SE3JointStateSpace(std::shared_ptr<SE3StateSpace> _space, Joint* _joint)
    : JointStateSpace(_space, _joint)
  {
  }

  void getState(StateSpace::State* _state) override
  {
    static_cast<SE3StateSpace*>(mStateSpace.get())->setIsometry(
      static_cast<SE3StateSpace::State*>(_state),
        FreeJoint::convertToTransform(mJoint->getPositions()));
  }

  void setState(const StateSpace::State* _state) override
  {
    mJoint->setPositions(FreeJoint::convertToPositions(
      static_cast<SE3StateSpace*>(mStateSpace.get())->getIsometry(
        static_cast<const SE3StateSpace::State*>(_state))));
  }
};


//=============================================================================
std::unique_ptr<JointStateSpace> createStateSpace(Joint* _joint)
{
  // Create a StateSpace for the Joint.
  if (isJointOfType<RevoluteJoint>(_joint))
  {
    if (_joint->isCyclic(0))
      return make_unique<SO2JointStateSpace>(
        std::make_shared<SO2StateSpace>(), _joint);
    else
      return make_unique<RealVectorJointStateSpace>(
        std::make_shared<RealVectorStateSpace>(1), _joint);
  }
  else if (isJointOfType<PrismaticJoint>(_joint))
  {
    return make_unique<RealVectorJointStateSpace>(
      std::make_shared<RealVectorStateSpace>(1), _joint);
  }
  else if (isJointOfType<TranslationalJoint>(_joint))
  {
    return make_unique<RealVectorJointStateSpace>(
      std::make_shared<RealVectorStateSpace>(3), _joint);
  }
  else if (isJointOfType<FreeJoint>(_joint))
  {
    return make_unique<SE3JointStateSpace>(
      std::make_shared<SE3StateSpace>(), _joint);
  }
  // TODO: Handle PlanarJoint, BallJoint, WeldJoint, and EulerJoint.
  else
  {
    std::stringstream msg;
    msg << "Joint '" << _joint->getName() << "' has unsupported type '"
         << _joint->getType() << "'.";
    throw std::runtime_error(msg.str());
  }
}

//=============================================================================
std::vector<std::unique_ptr<JointStateSpace>> createStateSpace(
  MetaSkeleton& _metaskeleton)
{
  std::vector<std::unique_ptr<JointStateSpace>> wrappers;
  wrappers.reserve(_metaskeleton.getNumJoints());

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

    wrappers.emplace_back(createStateSpace(joint));
  }

  return std::move(wrappers);
}

} // namespace

//=============================================================================
JointStateSpace::JointStateSpace(
      std::shared_ptr<StateSpace> _space, dart::dynamics::Joint* _joint)
  : mStateSpace(std::move(_space))
  , mJoint(_joint)
{
}

//=============================================================================
const std::shared_ptr<StateSpace>& JointStateSpace::getStateSpace()
{
  return mStateSpace;
}

//=============================================================================
dart::dynamics::Joint* JointStateSpace::getJoint()
{
  return mJoint;
}

//=============================================================================
MetaSkeletonStateSpace::MetaSkeletonStateSpace(
      MetaSkeletonPtr _metaskeleton,
      std::vector<StateSpacePtr> _stateSpaces,
      std::vector<std::unique_ptr<JointStateSpace>> _jointSpaces)
  : CompoundStateSpace(_stateSpaces)
  , mMetaSkeleton(std::move(_metaskeleton))
  , mJointSpaces(std::move(_jointSpaces))
{
}

//=============================================================================
std::shared_ptr<MetaSkeletonStateSpace> MetaSkeletonStateSpace::create(
  MetaSkeletonPtr _metaskeleton)
{
  std::vector<std::unique_ptr<JointStateSpace>> jointSpaces
    = createStateSpace(*_metaskeleton);

  std::vector<StateSpacePtr> stateSpaces;
  stateSpaces.reserve(jointSpaces.size());

  for (const auto& jointSpace : jointSpaces)
    stateSpaces.push_back(jointSpace->getStateSpace());

  return std::shared_ptr<MetaSkeletonStateSpace>(new MetaSkeletonStateSpace(
    std::move(_metaskeleton), std::move(stateSpaces), std::move(jointSpaces)));
}

//=============================================================================
MetaSkeletonPtr MetaSkeletonStateSpace::getMetaSkeleton() const
{
  return mMetaSkeleton;
}

//=============================================================================
void MetaSkeletonStateSpace::getStateFromMetaSkeleton(State* _state) const
{
  for (size_t ijoint = 0; ijoint < mJointSpaces.size(); ++ijoint)
    mJointSpaces[ijoint]->getState(getSubState(_state, ijoint));
}

//=============================================================================
auto MetaSkeletonStateSpace::getScopedStateFromMetaSkeleton() const
  -> ScopedState
{
  auto scopedState = createState();
  getStateFromMetaSkeleton(scopedState.getState());
  return std::move(scopedState);
}

//=============================================================================
void MetaSkeletonStateSpace::setStateOnMetaSkeleton(const State* _state)
{
  for (size_t ijoint = 0; ijoint < mJointSpaces.size(); ++ijoint)
    mJointSpaces[ijoint]->setState(getSubState(_state, ijoint));
}

} // namespace statespace
} // namespace aikido
