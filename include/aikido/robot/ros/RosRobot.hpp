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
  /// Construct a new RosRobot object.
  /// \param[in] urdf The path to the robot's URDF.
  /// \param[in] srdf The path to the robot's SRDF.
  /// \param[in] name The name of the robot.
  /// \param[in] retriever Retriever to access resources (urdf/srdf).
  /// \param[in] rosControllerManagerServerName The name of the controller manager server.
  /// \param[in] rosJointModeServerName The name of the joint mode controller actionlib client.
  RosRobot(
    const dart::common::Uri& urdf,
    const dart::common::Uri& srdf,
    const std::string name,
    const dart::common::ResourceRetrieverPtr& retriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>(),
    const std::string rosControllerManagerServerName = "",
    const std::string rosJointModeServerName = "");

  ///
  /// Construct a new RosRobot as subrobot.
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
  /// Registers a subset of the joints of the skeleton as a new RosRobot.
  /// Must be disjoint from other subrobots.
  ///
  /// \param[in] metaSkeleton The referential skeleton corresponding to the subrobot.
  /// \param[in] name Name of the subrobot.
  RobotPtr registerSubRobot(
      dart::dynamics::ReferentialSkeletonPtr refSkeleton,
      const std::string& name) override;

  ///
  /// Stops the controller corresponding to this executor and deactivates executor.
  void deactivateExecutor() override;

  ///
  /// Loads the controller. Throws a runtime_error if controller cannot be loaded.
  /// Add an (executor, controller, control mode) to the inactive (executor, controller, control mode) list
  /// Releases DoFs held by executor if held.
  /// 
  /// \param[in] executor The Executor to add to the inactive list.
  /// \param[in] controllerName The corresponding controller to load and add to the inactive list.
  /// \param[in] controllerMode The corresponding control mode to add to the inactive list.
  /// \return A robot-unique non-negative ID (negative implies failure)
  int registerExecutor(
      aikido::control::ExecutorPtr executor, 
      std::string controllerName, 
      hardware_interface::JointCommandModes controllerMode);

  ///
  /// Loads the controller. Throws a runtime_error if controller cannot be loaded.
  /// Add an (executor, controller) to the inactive (executor, controller, control mode) list
  /// Operations on this executor do not affect the control mode
  /// Releases DoFs held by executor if held.
  /// 
  /// \param[in] executor The Executor to add to the inactive list.
  /// \param[in] controllerName The corresponding controller to load and add to the inactive list.
  /// \return A robot-unique non-negative ID (negative implies failure)
  int registerExecutor(
      aikido::control::ExecutorPtr executor, 
      std::string controllerName);

  ///
  /// Add an executor to the inactive executors list.
  /// Operations on this executor do not afffect the controller or control mode.
  /// Releases DoFs held by executor if held.
  ///
  /// \param[in] executor The Executor to add to the inactive list.
  /// \return A robot-unique non-negative ID (negative implies failure)
  int registerExecutor(aikido::control::ExecutorPtr executor) override;

  ///
  /// Deactivates the current active executor.
  /// Starts the corresponding controller and switches to corresponding control mode.
  /// Sets an executor from the inactive executor list to be the active executor.
  /// Holds and releases DoFs as needed.
  ///
  /// \param[in] id of executor on executor list
  /// \return True if successful. If false, all executors are inactive.
  bool activateExecutor(int id) override;

  ///
  /// Convenience:
  /// Activates the *most recently registered* executor
  /// of the given type.
  ///
  /// \param[in] type of executor to activate.
  /// \return True if successful. If false, all executors are inactive.  
  bool activateExecutor(const aikido::control::ExecutorType type) override; // Rajat doubt: is this required?

  ///
  /// Starts the controller with name startControllerName.
  ///
  /// \param[in] startControllerName name of controller to start.
  /// \return True if successful. Otherwise false.  
  bool startController(const std::string startControllerName);

  ///
  /// Stops the controller with name stopControllerName.
  ///
  /// \param[in] stopControllerName name of controller to stop.
  /// \return True if successful. Otherwise false. 
  bool stopController(const std::string stopControllerName);

  ///
  /// Stops the controller with name stopControllerName and starts the controller with name startControllerName.
  ///
  /// \param[in] startControllerName name of controller to start.
  /// \param[in] stopControllerName name of controller to stop.
  /// \return True if successful. Otherwise false. 
  bool switchController(const std::string startControllerName, const std::string stopControllerName);

  ///
  /// Loads the controller with name loadControllerName.
  ///
  /// \param[in] loadControllerName name of controller to load.
  /// \return True if successful. Otherwise false. 
  bool loadController(const std::string loadControllerName);

  ///
  /// Switches the controller mode to jointMode.
  ///
  /// \param[in] jointMode control mode to switch to.
  /// \return True if successful. Otherwise false. 
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
