#ifndef AIKIDO_ROBOT_ROS_ROSROBOT_HPP_
#define AIKIDO_ROBOT_ROS_ROSROBOT_HPP_

#include <string>

#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <dart/utils/urdf/DartLoader.hpp>

#include "aikido/control/ros/RosJointModeCommandClient.hpp"
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
  /// Construct a new RosRobot object.
  /// \param[in] urdf The path to the robot's URDF.
  /// \param[in] srdf The path to the robot's SRDF.
  /// \param[in] name The name of the robot.
  /// \param[in] retriever Retriever to access resources (urdf/srdf).
  /// \param[in] rosControllerManagerServerName The name of the controller
  ///            manager server.
  /// \param[in] rosJointModeServerName The name of the joint
  ///            mode controller actionlib client.
  RosRobot(
      const dart::common::Uri& urdf,
      const dart::common::Uri& srdf,
      const std::string name,
      const bool addDefaultExecutors,
      const dart::common::ResourceRetrieverPtr& retriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>(),
      const ::ros::NodeHandle& node = ::ros::NodeHandle());

  ///
  /// Construct a new RosRobot as subrobot.
  /// Documentation Inherited
  RosRobot(
      dart::dynamics::ReferentialSkeletonPtr refSkeleton,
      Robot* rootRobot,
      dart::collision::CollisionDetectorPtr collisionDetector,
      std::shared_ptr<dart::collision::BodyNodeCollisionFilter> collisionFilter,
      const std::string name)
    : Robot(refSkeleton, rootRobot, collisionDetector, collisionFilter, name)
  {
  }

  /// Destructor.
  virtual ~RosRobot() = default;

  ///
  /// Steps the ros robot (and underlying executors and subrobots) through time.
  /// Call regularly to update the state of the ros robot.
  ///
  /// \param[in] timepoint The point in time to step to.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  ///
  /// Registers a subset of the joints of the skeleton as a new RosRobot.
  /// Must be disjoint from other subrobots.
  ///
  /// \param[in] metaSkeleton The referential skeleton corresponding to the
  /// subrobot. \param[in] name Name of the subrobot.
  RobotPtr registerSubRobot(
      dart::dynamics::ReferentialSkeletonPtr refSkeleton,
      const std::string& name) override;

  ///
  /// Stops the controller corresponding to this executor and deactivates
  /// executor. Throws a runtime_error if controller cannot be stopped.
  void deactivateExecutor() override;

  ///
  /// Loads the controller. Warns if controller cannot be
  /// loaded. Adds an (executor, controller, control mode) to the inactive
  /// (executor, controller, control mode) list. Releases DoFs held by executor
  /// if held.
  ///
  /// \param[in] executor The Executor to add to the inactive list.
  /// \param[in] desiredName The desired name for the executor.
  /// \param[in] controllerName The corresponding controller to load and add to
  /// the inactive list. 
  /// \param[in] controllerMode The corresponding control mode to add to the 
  /// inactive list. 
  /// \return A robot-unique non-negative ID (negative implies failure).
  int registerExecutor(
      aikido::control::ExecutorPtr executor,
      std::string desiredName,
      std::string controllerName,
      hardware_interface::JointCommandModes controllerMode);

  ///
  /// Loads the controller. Warns if controller cannot be
  /// loaded. Adds an (executor, controller) to the inactive (executor,
  /// controller, control mode) list. Operations on this executor do not affect
  /// the control mode. Releases DoFs held by executor if held.
  ///
  /// \param[in] executor The Executor to add to the inactive list.
  /// \param[in] desiredName The desired name for the executor.
  /// \param[in] controllerName The corresponding controller to load and add to
  /// the inactive list. 
  /// \return A robot-unique non-negative ID (negative implies failure).
  int registerExecutor(
      aikido::control::ExecutorPtr executor, 
      std::string desiredName,
      std::string controllerName);

  ///
  /// Deactivates the current active executor.
  /// Starts the corresponding controller and switches to corresponding control
  /// mode. Sets an executor from the inactive executor list to be the active
  /// executor. Holds and releases DoFs as needed.
  ///
  /// \param[in] id of executor on executor list
  /// \return True if successful. If false, all executors are inactive.
  bool activateExecutor(const int id) override;

  // This un-hides Robot::activateExecutor(const aikido::control::ExecutorType type) and Robot::activateExecutor(std::string id)
  using Robot::activateExecutor; 

  ///
  /// Sets the ros controller service client which enables controller loading.
  ///
  /// \param[in] rosControllerServiceClient ros controller service client.
  void setRosLoadControllerServiceClient(
      const std::shared_ptr<::ros::ServiceClient>& rosLoadControllerServiceClient); 

  ///
  /// Sets the ros controller service client which enables controller switching.
  ///
  /// \param[in] rosControllerServiceClient ros controller service client.
  void setRosSwitchControllerServiceClient(
      const std::shared_ptr<::ros::ServiceClient>& rosSwitchControllerServiceClient);

  ///
  /// Sets the ros joint mode command client which enables controller mode switching.
  ///
  /// \param[in] rosJointModeCommandClient ros joint mode command client.
  void setRosJointModeCommandClient(
      const std::shared_ptr<aikido::control::ros::RosJointModeCommandClient>& rosJointModeCommandClient);

protected:
  std::unordered_map<int, hardware_interface::JointCommandModes> mRosControllerModes;
  std::unordered_map<int, std::string> mRosControllerNames;

  // Ros node associated with this robot.
  const ::ros::NodeHandle mNode;

  // Ros load controller service client.
  std::shared_ptr<::ros::ServiceClient> mRosLoadControllerServiceClient;

  // Ros switch controller service client.
  std::shared_ptr<::ros::ServiceClient> mRosSwitchControllerServiceClient;

  // Ros joint mode command client.
  // Default is nullptr so mode switching is impossible.
  std::shared_ptr<aikido::control::ros::RosJointModeCommandClient>
      mRosJointModeCommandClient; 

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
  /// Stops the controller with name stopControllerName and starts the
  /// controller with name startControllerName.
  ///
  /// \param[in] startControllersNames list of names of controllers to start.
  /// \param[in] stopControllersNames list of names of controllers to stop.
  /// \return True if successful. Otherwise false.
  bool switchControllers(
      const std::vector<std::string> startControllersNames, 
      const std::vector<std::string> stopControllersNames);

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
  bool switchControllerMode(
      const hardware_interface::JointCommandModes jointMode);
};

} // namespace ros
} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_ROS_ROSROBOT_HPP_
