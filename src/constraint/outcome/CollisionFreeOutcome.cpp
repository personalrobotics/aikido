#include <aikido/constraint/outcome/CollisionFreeOutcome.hpp>

namespace aikido {
namespace constraint {

//==============================================================================
bool CollisionFreeOutcome::isSatisfied() const
{
  if (mCollisionBodyNodes.size() > 0 || mSelfCollisionBodyNodes.size() > 0)
  {
    return false;
  }

  return true;
}

//==============================================================================
std::string CollisionFreeOutcome::toString() const
{
  std::stringstream ss;
  ss << "ALL COLLISIONS: " << std::endl;

  for (auto collBodyNodeName : mCollisionBodyNodes)
  {
    ss << "[COLLISION]: " << collBodyNodeName << std::endl;
  }

  for (auto selfCollBodyNodeName : mSelfCollisionBodyNodes)
  {
    ss << "[SELF COLLISION]: " << selfCollBodyNodeName << std::endl;
  }

  return std::move(ss.str());
}

//==============================================================================
void CollisionFreeOutcome::markCollisionBodyNode(
    const std::string& bodyNodeName)
{
  mCollisionBodyNodes.push_back(bodyNodeName);
}

//==============================================================================
void CollisionFreeOutcome::markSelfCollisionBodyNode(
    const std::string& bodyNodeName)
{
  mSelfCollisionBodyNodes.push_back(bodyNodeName);
}

} // namespace constraint
} // namespace aikido
