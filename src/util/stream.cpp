#include <aikido/util/stream.hpp>

namespace aikido {
namespace util {

void printCollisionObject(
    const dart::collision::CollisionObject& collisionObject,
    std::ostream& stream)
{
  const auto shapeFrame = collisionObject.getShapeFrame();

  if (const auto shapeNode = shapeFrame->asShapeNode())
  {
    const auto bodyNode = shapeNode->getBodyNodePtr();
    const auto skeleton = bodyNode->getSkeleton();
    stream << skeleton->getName() << ":" << bodyNode->getName() << ":"
           << shapeNode->getName();
  }
  else
  {
    stream << "ShapeFrame::" << shapeFrame->getName();
  }
}

void printCollisionResult(
    const dart::collision::CollisionResult& collisionReport,
    std::ostream& stream)
{
  if (collisionReport.getNumContacts() > 0)
  {
    const auto& contact = collisionReport.getContact(0);

    stream << "<";

    if (contact.collisionObject1)
      printCollisionObject(*contact.collisionObject1, stream);
    else
      stream << "unknown";

    stream << " x ";

    if (contact.collisionObject2)
      printCollisionObject(*contact.collisionObject2, stream);
    else
      stream << "unknown";

    stream << ">";
  }
  else
  {
    stream << "<none>";
  }
}

namespace operators {

std::ostream& operator<<(
    std::ostream& stream,
    const dart::collision::CollisionObject& collisionObject)
{
  printCollisionObject(collisionObject, stream);
  return stream;
}

std::ostream& operator<<(
    std::ostream& stream,
    const dart::collision::CollisionResult& collisionResult)
{
  printCollisionResult(collisionResult, stream);
  return stream;
}

}  // namespace operators
}  // namespace util
}  // namespace aikido
