#include <aikido/statespace/dart/JointStateSpace.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
JointStateSpace::JointStateSpace(dart::dynamics::Joint* _joint)
  : mJoint(_joint)
{
  assert(_joint);
}

//=============================================================================
dart::dynamics::Joint* JointStateSpace::getJoint() const
{
  return mJoint;
}

} // namespace statespace
} // namespace aikido
