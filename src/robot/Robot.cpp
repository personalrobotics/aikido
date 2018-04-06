#include "aikido/robot/Robot.hpp"

namespace aikido {
namespace robot {

//==============================================================================
dart::dynamics::MetaSkeletonPtr Robot::getMetaSkeleton()
{
  return std::const_pointer_cast<dart::dynamics::MetaSkeleton>(
      const_cast<const Robot*>(this)->getMetaSkeleton());
}

} // namespace robot
} // namespace aikido
