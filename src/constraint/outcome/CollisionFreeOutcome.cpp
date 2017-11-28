#include <aikido/constraint/outcome/CollisionFreeOutcome.hpp>

namespace aikido {
namespace constraint {

//==============================================================================
bool CollisionFreeOutcome::isSatisfied() const
{
  return mCollisionBodyNodes.empty() && mSelfCollisionBodyNodes.empty();
}

//==============================================================================
std::string CollisionFreeOutcome::toString() const
{
  std::stringstream ss;
  ss << "ALL COLLISIONS: " << std::endl;

  for (const auto& collBodyNodeName : mCollisionBodyNodes)
  {
    ss << "[COLLISION]: " << collBodyNodeName << std::endl;
  }

  for (const auto& selfCollBodyNodeName : mSelfCollisionBodyNodes)
  {
    ss << "[SELF COLLISION]: " << selfCollBodyNodeName << std::endl;
  }

  return ss.str();
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
