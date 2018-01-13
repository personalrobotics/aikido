#ifndef AIKIDO_ROBOT_MOBILEBASE_HPP_
#define AIKIDO_ROBOT_MOBILEBASE_HPP_

#include <dart/dart.hpp>
#include <string>
#include <aikido/robot/Robot.hpp>

namespace aikido {
namespace robot {

/// A base class for manipulator robots
class MobileBase : public Robot
{
public:

  /// Create a new Robot
  /// \param name Name for the new Robot
  virtual static std::unique_ptr<Robot> create(const std::string &name) override;

  /// Clones this Robot.
  /// \param newName New name for this robot
  virtual std::unique_ptr<Robot> clone(const std::string& newName) override;

protected:

};

} // namespace robot
} // namespace aikido
