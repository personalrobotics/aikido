#include "aikido/robot/Robot.hpp"

namespace aikido {
namespace robot {

//==============================================================================
dart::dynamics::MetaSkeletonPtr Robot::getMetaSkeleton()
{
  return std::const_pointer_cast<dart::dynamics::MetaSkeleton>(
      const_cast<const Robot*>(this)->getMetaSkeleton());
}

//==============================================================================
statespace::dart::MetaSkeletonStateSpacePtr Robot::getStateSpace()
{
  return std::const_pointer_cast<statespace::dart::MetaSkeletonStateSpace>(
      const_cast<const Robot*>(this)->getStateSpace());
}

} // namespace robot
} // namespace aikido
