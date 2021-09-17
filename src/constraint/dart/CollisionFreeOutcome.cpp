#include "aikido/constraint/dart/CollisionFreeOutcome.hpp"

#include <sstream>

namespace aikido {
namespace constraint {
namespace dart {

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
void CollisionFreeOutcome::clear()
{
  mPairwiseContacts.clear();
  mSelfContacts.clear();
}

//==============================================================================
std::vector<::dart::collision::Contact>
CollisionFreeOutcome::getPairwiseContacts() const
{
  return mPairwiseContacts;
}

//==============================================================================
std::vector<::dart::collision::Contact> CollisionFreeOutcome::getSelfContacts()
    const
{
  return mSelfContacts;
}

//==============================================================================
std::string CollisionFreeOutcome::getCollisionObjectName(
    const ::dart::collision::CollisionObject* object) const
{
  const ::dart::dynamics::ShapeFrame* frame = object->getShapeFrame();

  if (frame->isShapeNode())
  {
    const ::dart::dynamics::ShapeNode* node = frame->asShapeNode();
    return node->getBodyNodePtr()->getName();
  }
  else
  {
    return frame->getName();
  }
}

} // namespace dart
} // namespace constraint
} // namespace aikido
