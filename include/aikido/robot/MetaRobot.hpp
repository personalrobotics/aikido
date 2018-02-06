#ifndef AIKIDO_ROBOT_METAROBOT_HPP_
#define AIKIDO_ROBOT_METAROBOT_HPP_

#include <string>
#include <unordered_map>
#include <dart/dart.hpp>
#include <aikido/robot/Robot.hpp>

namespace aikido {
namespace robot {

/// A base class for a meta robot which has a list of robots
class MetaRobot : public Robot
{
public:

  /// Clones this Robot.
  /// \param newName New name for this robot
  virtual std::unique_ptr<Robot> clone(const std::string &newName) override;

  /// \return the named robot or nullptr if it doesn't exist
  std::unique_ptr<Robot> getRobot(const std::string &name);

protected:

  std::unordered_map<std::string, Robot> mRobots;
};

using MetaRobotPtr = std::shared_ptr<MetaRobot>;

} // namespace robot
} // namespace aikido
