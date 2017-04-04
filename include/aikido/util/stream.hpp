#ifndef AIKIDO_UTIL_STREAM_HPP_
#define AIKIDO_UTIL_STREAM_HPP_
#include <iostream>
#include <dart/collision/collision.hpp>

namespace aikido {
namespace util {

/// Prints a CollisionObject for debugging purposes.
/// \param collisionObject object to print
/// \param[out] stream to use for output
void printCollisionObject(
    const dart::collision::CollisionObject& collisionObject,
    std::ostream& stream);

/// Prints a CollisionResult for debugging purposes.
/// \param collisionResult result to print
/// \param[out] stream to use for output
void printCollisionResult(
    const dart::collision::CollisionResult& collisionResult,
    std::ostream& stream);

namespace operators {

/// Operator that calls printCollisionObject.
std::ostream& operator<<(
    std::ostream& stream,
    const dart::collision::CollisionObject& collisionObject);

/// Operator that calls printCollisionReport.
std::ostream& operator<<(
    std::ostream& stream,
    const dart::collision::CollisionResult& collisionResult);

} // namespace operators

} // namespace util
} // namespace aikido

#endif // ifndef AIKIDO_UTIL_STREAM_HPP_
