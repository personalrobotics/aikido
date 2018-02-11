#include "aikido/robot/Hand.hpp"

namespace aikido {
namespace robot {

//==============================================================================
Hand::Hand(const std::string &name,
    dart::dynamics::BranchPtr hand,
    dart::dynamics::BodyNode* endEffectorBodyNode)
: mName(name),
: mHand(hand),
: mEndEffectorBodyNode(endEffectorBodyNode)
{
}

//==============================================================================
Hand::~Hand()
{
  // Do nothing
}

//==============================================================================
dart::dynamics::BodyNode* Hand::getBodyNode()
{
  return mEndEffectorBodyNode.get();
}

//==============================================================================
dart::dynamics::BranchPtr Hand::getMetaSkeleton()
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
}

//==============================================================================
void Hand::ungrab()
{
}

//==============================================================================
void Hand::executePreshape(const std::string &preshapeName)
{
  boost::optional<Eigen::VectorXd> preshape = getPreshape(preshapeName);

  if (!preshape)
  {
    std::stringstream message;
    message << "[Hand::executePreshape] Unknown preshape name '"
            << preshapeName << "' specified.";
    throw std::runtime_error(message.str());
  }

  executor->execute(preshape.get());
}

//==============================================================================
void Hand::step()
{
  mExecutor->step();
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
      auto jointIndex
          = jointNameToPositionIndexMap.find(jointName);
      if (jointIndex == fingerJointNameToPositionIndexMap.end())
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

