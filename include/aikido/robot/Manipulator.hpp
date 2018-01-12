#ifndef AIKIDO_ROBOT_MANIPULATOR_HPP_
#define AIKIDO_ROBOT_MANIPULATOR_HPP_

#include <dart/dart.hpp>
#include <string>
#include <aikido/robot/Robot.hpp>
#include <aikido/robot/Hand.hpp>

namespace aikido {
namespace robot {

/// A base class for manipulator robots
class Manipulator : public Robot
{
public:

  /// Clones this Robot.
  /// \param newName New name for this robot
  virtual std::unique_ptr<Robot> clone(const std::string& newName) override;

  std::unique_ptr<aikido::robot::Hand> getHand();

  dart::dynamics::MetaSkeletonPtr getArm();


protected:

}

using ManipulatorPtr = std::shared_ptr<Manipulator>;

} // namespace robot
} // namespace aikido
