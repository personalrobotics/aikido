#ifndef AIKIDO_ROBOT_METAROBOT_HPP_
#define AIKIDO_ROBOT_METAROBOT_HPP_

#include <string>
#include <unordered_map>
#include <dart/dart.hpp>
#include "aikido/robot/Robot.hpp"

namespace aikido {
namespace robot {

/// A base class for a meta robot which has a list of robots
class CompositeRobot : public Robot
{
public:

  CompositeRobot(std::unordered_map<std::string, std::unique_ptr<Robot>> robots);

  /// Clones this Robot.
  /// \param newName New name for this robot
  virtual std::unique_ptr<Robot> clone(const std::string &newName) override;

  /// \return the named robot or nullptr if it doesn't exist
  std::unique_ptr<Robot> getRobot(const std::string &name);

protected:

  std::unordered_map<std::string, std::unique_ptr<Robot>> mRobots;
};

using CompositeRobotPtr = std::shared_ptr<CompositeRobot>;

} // namespace robot
} // namespace aikido

#endif

