#ifndef AIKIDO_ROBOT_MANIPULATOR_HPP_
#define AIKIDO_ROBOT_MANIPULATOR_HPP_

#include <dart/dart.hpp>
#include <string>
#include <aikido/robot/Robot.hpp>

namespace aikido {
namespace robot {

/// A base class for manipulator robots
class Manipulator : public Robot
{
public:

  /// Create a new Robot
  /// \param name Name for the new Robot
  virtual static std::unique_ptr<Robot> create(const std::string &name) = 0;

  /// Clones this Robot.
  /// \param newName New name for this robot
  virtual std::unique_ptr<Robot> clone(const std::string& newName) = 0;

protected:

  /// Name of this robot
  std::string mName;
};

} // namespace robot
} // namespace aikido
