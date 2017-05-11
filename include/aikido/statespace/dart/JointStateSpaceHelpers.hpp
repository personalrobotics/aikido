#ifndef AIKIDO_STATESPACE_DART_JOINTSTATESPACEHELPERS_HPP_
#define AIKIDO_STATESPACE_DART_JOINTSTATESPACEHELPERS_HPP_
#include <memory>
#include <dart/dynamics/dynamics.hpp>
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// Create a JointStateSpace for a Joint whose type is known at compile time.
///
/// \tparam JointType joint type
/// \param _joint joint to create a state space for
/// \return state space of \c _joint
template <class JointType>
std::unique_ptr<JointStateSpace> createJointStateSpaceFor(JointType* _joint);

/// Create a JointStateSpace for an aribtrary Joint. This may be significantly
/// slower than createJointStateSpaceFor(), so only use this function if the
/// Joint's type is not known at compile time.
///
/// \param _joint joint to create a state space for
/// \return state space of \c _joint
std::unique_ptr<JointStateSpace> createJointStateSpace(
    ::dart::dynamics::Joint* _joint);

} // namespace dart
} // namespace statespace
} // namespace aikido

#include "detail/JointStateSpaceHelpers-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_DART_JOINTSTATESPACEHELPERS_HPP_
