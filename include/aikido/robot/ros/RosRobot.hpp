#ifndef AIKIDO_ROBOT_ROS_ROSROBOT_HPP_
#define AIKIDO_ROBOT_ROS_ROSROBOT_HPP_

#include <string>

#include <dart/utils/urdf/DartLoader.hpp>

#include "aikido/io/CatkinResourceRetriever.hpp"
#include "aikido/robot/Robot.hpp"

namespace aikido {
namespace robot {
namespace ros {

AIKIDO_DECLARE_POINTERS(RosRobot)

/// Robot interface augmented with ROS components
class RosRobot : public Robot
{
public:
  /// Construct a new Robot object.
  /// \param[in] urdf The path to the robot's URDF.
  /// \param[in] srdf The path to the robot's SRDF.
  /// \param[in] name The name of the robot.
  /// \param[in] retriever Retriever to access resources (urdf/srdf).
  RosRobot(
      const dart::common::Uri& urdf,
      const dart::common::Uri& srdf,
      const std::string name,
      const dart::common::ResourceRetrieverPtr& retriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>());

  /// Destructor.
  virtual ~RosRobot() = default;
};

} // namespace ros
} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_ROS_ROSROBOT_HPP_
