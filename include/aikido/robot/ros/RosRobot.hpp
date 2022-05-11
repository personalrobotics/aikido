#ifndef AIKIDO_ROBOT_ROS_ROSROBOT_HPP_
#define AIKIDO_ROBOT_ROS_ROSROBOT_HPP_

#include <string>

#include <dart/utils/urdf/DartLoader.hpp>

#include "aikido/io/CatkinResourceRetriever.hpp"
#include "aikido/robot/Robot.hpp"
#include "aikido/control/ros/RosJointModeCommandClient.hpp"

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>

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
      = std::make_shared<aikido::io::CatkinResourceRetriever>(),
    const std::string rosControllerManagerServerName = "",
    const std::string rosJointModeServerName = "");

  ///
  /// Construct a new Robot as subrobot.
  ///
  /// \param[in] refSkeleton The metaskeleton defining the robot.
  /// \param[in] rootRobot Pointer to parent robot
  /// \param[in] collisionDetector Parent robot's collision detector
  /// \param[in] collisionFilter Parent robot's collision filter
  /// \param[in] name Unique Name of sub-robot
  RosRobot(
    dart::dynamics::ReferentialSkeletonPtr refSkeleton,
    Robot* rootRobot,
    dart::collision::CollisionDetectorPtr collisionDetector,
    std::shared_ptr<dart::collision::BodyNodeCollisionFilter> collisionFilter,
    const std::string name)
  : Robot(refSkeleton, rootRobot,collisionDetector, collisionFilter, name)
  {} 

  /// Destructor.
  virtual ~RosRobot() = default;

  ///
  /// Registers a subset of the joints of the skeleton as a new robot.
  /// Must be disjoint from other subrobots.
  ///
  /// \param[in] metaSkeleton The referential skeleton corresponding to the
  /// subrobot. \param[in] name Name of the subrobot.
  RobotPtr registerSubRobot(
      dart::dynamics::ReferentialSkeletonPtr refSkeleton,
      const std::string& name) override;

  //==============================================================================
  void deactivateExecutor() override;

  //==============================================================================
  int registerExecutor(
      aikido::control::ExecutorPtr executor, 
      std::string controller_name, 
      hardware_interface::JointCommandModes controller_mode);
      // Rajat TODO: Can extract controller name from executor server name?

  //==============================================================================
  int registerExecutor(aikido::control::ExecutorPtr executor) override;

  //==============================================================================
  bool activateExecutor(int id) override;

  //==============================================================================
  bool activateExecutor(const aikido::control::ExecutorType type) override; // Rajat doubt: is this required?

  //==============================================================================
  bool startController(const std::string startControllerName);

  //==============================================================================
  bool stopController(const std::string stopControllerName);

  //==============================================================================
  bool switchController(const std::string startControllerName, const std::string stopControllerName);

  //==============================================================================
  bool loadController(const std::string loadControllerName);

  //==============================================================================
  bool switchControllerMode(const hardware_interface::JointCommandModes jointMode);


protected: 

  std::vector<hardware_interface::JointCommandModes> mRosControllerModes;
  std::vector<std::string> mRosControllerNames;

  // Ros node associated with this robot.
  std::unique_ptr<::ros::NodeHandle> mNode;

  // Ros controller service client.
  std::unique_ptr<::ros::ServiceClient> mRosControllerServiceClient;

  // Ros joint mode command client.
  std::unique_ptr<aikido::control::ros::RosJointModeCommandClient> mRosJointModeCommandClient; //default is null ptr so mode switching is impossible

};

} // namespace ros
} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_ROS_ROSROBOT_HPP_
