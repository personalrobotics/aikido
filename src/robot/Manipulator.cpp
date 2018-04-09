#include "aikido/robot/Manipulator.hpp"

namespace aikido {
namespace robot {

//==============================================================================
HandPtr Manipulator::getHand()
{
  return std::const_pointer_cast<Hand>(
      const_cast<const Manipulator*>(this)->getHand());
}

} // namespace robot
} // namespace aikido
