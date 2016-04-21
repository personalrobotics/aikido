#include <aikido/statespace/dart/JointStateSpace.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//=============================================================================
JointStateSpace::JointStateSpace(::dart::dynamics::Joint* _joint)
  : mJoint(_joint)
{
  assert(_joint);
}

//=============================================================================
::dart::dynamics::Joint* JointStateSpace::getJoint() const
{
  return mJoint;
}

} // namespace dart
} // namespace statespace
} // namespace aikido
