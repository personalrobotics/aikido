#include <aikido/constraint/outcome/CollisionFreeOutcome.hpp>

namespace aikido {
namespace constraint {

//==============================================================================
bool CollisionFreeOutcome::isSatisfied() const
{
  return mPairwiseContacts.empty() && mSelfContacts.empty();
}

//==============================================================================
std::string CollisionFreeOutcome::toString() const
{
  std::stringstream ss;
  ss << "ALL COLLISIONS: " << std::endl;

  for (const auto& pairwiseContact : mPairwiseContacts)
  {
    ss << "[COLLISION]: "
       << getCollisionObjectName(pairwiseContact.collisionObject1) << " and "
       << getCollisionObjectName(pairwiseContact.collisionObject2) << std::endl;
  }

  for (const auto& selfContact : mSelfContacts)
  {
    ss << "[SELF COLLISION]: "
       << getCollisionObjectName(selfContact.collisionObject1) << " and "
       << getCollisionObjectName(selfContact.collisionObject2) << std::endl;
  }

  return ss.str();
}

//==============================================================================
void CollisionFreeOutcome::markPairwiseContact(
    const dart::collision::Contact& pairwiseContact)
{
  mPairwiseContacts.push_back(pairwiseContact);
}

//==============================================================================
void CollisionFreeOutcome::markSelfContact(
    const dart::collision::Contact& selfContact)
{
  mSelfContacts.push_back(selfContact);
}

//==============================================================================
std::string CollisionFreeOutcome::getCollisionObjectName(
    dart::collision::CollisionObject* object) const
{
  const dart::dynamics::ShapeFrame* frame = object->getShapeFrame();

  if (frame->isShapeNode())
  {
    const dart::dynamics::ShapeNode* node = frame->asShapeNode();
    return node->getBodyNodePtr()->getName();
  }
  else
  {
    return frame->getName();
  }
}

} // namespace constraint
} // namespace aikido
