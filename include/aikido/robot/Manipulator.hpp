#ifndef AIKIDO_ROBOT_MANIPULATOR_HPP_
#define AIKIDO_ROBOT_MANIPULATOR_HPP_

#include <string>
#include <dart/dart.hpp>
#include "aikido/robot/Hand.hpp"
#include "aikido/robot/Robot.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(Manipulator)

/// A base class for a manipulator which has a hand.
class Manipulator : public Robot
{
public:
  virtual ~Manipulator() = default;

  virtual HandPtr getHand() = 0;
};

} // namespace robot
} // namespace aikido

#endif
