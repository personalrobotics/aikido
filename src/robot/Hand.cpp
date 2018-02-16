#include "aikido/robot/Hand.hpp"
#include <dart/common/StlHelpers.hpp>

namespace aikido {
namespace robot {

using dart::dynamics::BodyNodePtr;
using dart::dynamics::SkeletonPtr;

namespace {

void disablePairwiseSelfCollision(
    const BodyNodePtr& singleNode,
    const BodyNodePtr& rootNode,
    const std::shared_ptr<dart::collision::BodyNodeCollisionFilter>&
        selfCollisionFilter)
{
#ifndef NDEBUG
  std::cout << "Disabling collision between " << rootNode->getName() << " and "
            << singleNode->getName() << std::endl;
#endif

  selfCollisionFilter->addBodyNodePairToBlackList(rootNode, singleNode);
  for (std::size_t i = 0; i < rootNode->getNumChildBodyNodes(); ++i)
  {
    disablePairwiseSelfCollision(
        singleNode, rootNode->getChildBodyNode(i), selfCollisionFilter);
  }
}

void enablePairwiseSelfCollision(
    const BodyNodePtr& singleNode,
    const BodyNodePtr& rootNode,
    const std::shared_ptr<dart::collision::BodyNodeCollisionFilter>&
        selfCollisionFilter)
{
#ifndef NDEBUG
  std::cout << "Enabling collision between " << rootNode->getName() << " and "
            << singleNode->getName() << std::endl;
#endif
  selfCollisionFilter->removeBodyNodePairFromBlackList(rootNode, singleNode);
  for (std::size_t i = 0; i < rootNode->getNumChildBodyNodes(); ++i)
  {
    enablePairwiseSelfCollision(
        singleNode, rootNode->getChildBodyNode(i), selfCollisionFilter);
  }
}

} // namespace

//==============================================================================
Hand::Hand(
    const std::string& name,
    dart::dynamics::BranchPtr hand,
    bool simulation,
    dart::dynamics::BodyNode* endEffectorBodyNode,
    std::shared_ptr<aikido::control::PositionCommandExecutor> executor,
    std::unordered_map<std::string, size_t> fingerJointNameToPositionIndexMap)
  : mName(name)
  , mHand(hand)
  , mSimulation(simulation)
  , mEndEffectorBodyNode(endEffectorBodyNode)
  , mExecutor(executor)
  , mFingerJointNameToPositionIndexMap(fingerJointNameToPositionIndexMap)
  , mGrabMetadata(nullptr)
{
}

//==============================================================================
dart::dynamics::BodyNode* Hand::getBodyNode() const
{
  return mEndEffectorBodyNode.get();
}

//==============================================================================
dart::dynamics::BranchPtr Hand::getMetaSkeleton() const
{
  return mHand;
}

//==============================================================================
boost::optional<Eigen::Isometry3d> Hand::getEndEffectorTransform(
    const std::string& objectType) const
{
  auto transform = mEndEffectorTransforms.find(objectType);
  if (transform == mEndEffectorTransforms.end())
    return boost::none;

  return transform->second;
}

//==============================================================================
void Hand::grab(const dart::dynamics::SkeletonPtr& bodyToGrab)
{
  using dart::dynamics::Joint;
  using dart::dynamics::FreeJoint;
  using dart::dynamics::WeldJoint;

  // TODO: implement grabbing multiple objects

  // Check if end-effector is already grabbing object
  if (mGrabMetadata)
  {
    std::stringstream ss;
    // TODO: use proper logging
    ss << "[Hand::grab] An end effector may only grab one object."
       << " '" << mEndEffectorBodyNode->getName() << "' is grabbing '"
       << mGrabMetadata->mOldName << "'" << std::endl;

    throw std::runtime_error(ss.str());
  }

  // Assume the skeleton is a single pair of FreeJoint and BodyNode
  if (bodyToGrab->getNumBodyNodes() != 1)
  {
    std::stringstream ss;
    // TODO: use proper logging
    ss << "[Hand::grab] Only Skeletons with one BodyNode may be "
       << "grabbed. Skeleton '" << bodyToGrab->getName() << "' has "
       << bodyToGrab->getNumBodyNodes() << " BodyNodes" << std::endl;

    throw std::runtime_error(ss.str());
  }

  // TODO: this should be Skeleton::getRootJoint() once DART 6.2 is released
  Joint* joint = bodyToGrab->getJoint(0);
  FreeJoint* freeJoint = dynamic_cast<FreeJoint*>(joint);
  if (freeJoint == nullptr)
  {
    std::stringstream ss;
    // TODO: use proper logging
    ss << "[Hand::grab] Only Skeletons with a root FreeJoint may "
       << "be grabbed. Skeleton '" << bodyToGrab->getName() << "' has a "
       << "root " << joint->getType() << std::endl;

    throw std::runtime_error(ss.str());
  }

  // Get fields for GrabMetadata
  auto bodyNode = freeJoint->getChildBodyNode();
  std::string bodyNodeName = bodyNode->getName();
  FreeJoint::Properties jointProperties = freeJoint->getFreeJointProperties();

  // Get relative transform between end effector and BodyNode
  auto endEffectorToBodyTransform
      = bodyNode->getTransform(mEndEffectorBodyNode);

  // Connect grabbed BodyNode to end effector
  WeldJoint::Properties weldJointProperties;
  weldJointProperties.mT_ParentBodyToJoint = endEffectorToBodyTransform;
  bodyNode->moveTo<WeldJoint>(mEndEffectorBodyNode, weldJointProperties);

  // Moving the grabbed object into the same skeleton as the hand means that it
  // will be considered during self-collision checking. Therefore, we need to
  // disable self-collision checking between grabbed object and hand.
  disablePairwiseSelfCollision(
      bodyNode, mEndEffectorBodyNode, mSelfCollisionFilter);

  mGrabMetadata = dart::common::make_unique<GrabMetadata>(
      bodyNode, bodyNodeName, bodyToGrab, jointProperties);
}

//==============================================================================
void Hand::ungrab()
{
  using dart::dynamics::Joint;
  using dart::dynamics::FreeJoint;

  // Ensure end effector is already grabbing object
  if (!mGrabMetadata)
  {
    std::stringstream ss;

    // TODO: use proper logging
    ss << "[BarrettHand::ungrab] End effector \""
       << mEndEffectorBodyNode->getName() << "\" is not grabbing an object."
       << std::endl;
    throw std::runtime_error(ss.str());
  }

  // Get grabbed body node and its transform wrt the world
  BodyNodePtr grabbedBodyNode = mGrabMetadata->mBodyNode;
  Eigen::Isometry3d grabbedBodyTransform = grabbedBodyNode->getTransform();

  // Re-enable self-collision checking between grabbed object and hand
  enablePairwiseSelfCollision(
      grabbedBodyNode, mEndEffectorBodyNode, mSelfCollisionFilter);

  // Move grabbed BodyNode to root of the old object Skeleton
  SkeletonPtr skeleton = mGrabMetadata->mParentSkeleton;
  grabbedBodyNode->moveTo<FreeJoint>(
      skeleton, nullptr, mGrabMetadata->mJointProperties);

  // Set transform of skeleton FreeJoint wrt world
  Joint* joint = skeleton->getJoint(0);
  assert(joint != nullptr);
  FreeJoint* freeJoint = dynamic_cast<FreeJoint*>(joint);
  freeJoint->setTransform(grabbedBodyTransform);

  // Restore old name. If the skeleton of the grabbedBodyNode adds a body with
  // the name oldName while it is removed from the skeleton, then the object
  // cannot be named oldName when it is added back. Instead, DART will rename it
  // to something like oldName(1).
  std::string oldName = mGrabMetadata->mOldName;
  std::string newName = grabbedBodyNode->setName(oldName);
  if (newName != oldName)
  {
    // TODO: use proper logging (warn)
    std::cout << "[BarrettHand::ungrab] Released object was renamed from \""
              << oldName << "\" to \"" << newName << "\"" << std::endl;
  }

  mGrabMetadata.reset();
}

//==============================================================================
void Hand::executePreshape(const std::string& preshapeName)
{
  boost::optional<Eigen::VectorXd> preshape = getPreshape(preshapeName);

  if (!preshape)
  {
    std::stringstream message;
    message << "[Hand::executePreshape] Unknown preshape name '" << preshapeName
            << "' specified.";
    throw std::runtime_error(message.str());
  }

  mExecutor->execute(preshape.get());
}

//==============================================================================
void Hand::step(const std::chrono::system_clock::time_point& timepoint)
{
  mExecutor->step(timepoint);
}

//==============================================================================
void Hand::parseYAMLToPreshapes(const YAML::Node& node)
{
  for (const auto preshapeNode : node)
  {
    auto preshapeName = preshapeNode.first.as<std::string>();
    auto jointNodes = preshapeNode.second;

    // TODO: check
    Eigen::VectorXd preshape(4);
    for (auto joint : jointNodes)
    {
      auto jointName = joint.first.as<std::string>();
      auto jointIndex = mFingerJointNameToPositionIndexMap.find(jointName);
      if (jointIndex == mFingerJointNameToPositionIndexMap.end())
      {
        std::stringstream message;
        message << "Joint '" << jointName << "' does not exist." << std::endl;
        throw std::runtime_error(message.str());
      }
      preshape[jointIndex->second] = joint.second.as<double>();
    }

    mPreshapeConfigurations.emplace(preshapeName, preshape);
  }
}

//==============================================================================
void Hand::parseYAMLToEndEffectorTransforms(const YAML::Node& node)
{
  mEndEffectorTransforms = node[mName].as<EndEffectorTransformMap>();
}

//==============================================================================
boost::optional<Eigen::VectorXd> Hand::getPreshape(
    const std::string& preshapeName)
{
  auto preshape = mPreshapeConfigurations.find(preshapeName);
  if (preshape == mPreshapeConfigurations.end())
    return boost::none;

  return preshape->second;
}
}
}
