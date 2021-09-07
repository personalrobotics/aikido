#ifndef AIKIDO_ROBOT_CONCRETEMANIPULATOR_HPP_
#define AIKIDO_ROBOT_CONCRETEMANIPULATOR_HPP_

#include "aikido/robot/Hand.hpp"
#include "aikido/robot/Manipulator.hpp"
#include "aikido/robot/util.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(ConcreteManipulator)

/// A concrete implementation of a manipulator.
/// ConcreteManipulator supports a collection of
/// end-effector specific planning methods.
class ConcreteManipulator : public Manipulator
{
public:
  // Expose base class functions
  using Manipulator::getHand;
  using Robot::getMetaSkeleton;
  using Robot::getStateSpace;

  /// Constructor.
  /// \param[in] robot Robot corresponding to this manipulator.
  /// \param[in] hand Hand of this manipulator.
  ConcreteManipulator(RobotPtr robot, HandPtr hand);

  virtual ~ConcreteManipulator() = default;

  // Documentation inherited.
  virtual ConstHandPtr getHand() const override;

  // Documentation inherited.
  virtual std::future<void> executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) const override;

  // Documentation inherited.
  virtual boost::optional<Eigen::VectorXd> getNamedConfiguration(
      const std::string& name) const override;

  // Documentation inherited.
  virtual void setNamedConfigurations(
      std::unordered_map<std::string, const Eigen::VectorXd>
          namedConfigurations) override;

  // Documentation inherited.
  virtual std::string getName() const override;

  // Documentation inherited.
  virtual dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const override;

  // Documentation inherited.
  virtual aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
  getStateSpace() const override;

  // Documentation inherited.
  virtual void setRoot(Robot* robot) override;

  // Documentation inherited.
  virtual void step(
      const std::chrono::system_clock::time_point& timepoint) override;

  // Documentation inherited.
  virtual constraint::dart::CollisionFreePtr getSelfCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const override;

  // Documentation inherited.
  virtual aikido::constraint::TestablePtr getFullCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const constraint::dart::CollisionFreePtr& collisionFree) const override;

  /// TODO: Replace this with Problem interface.
  /// Plans to a desired end-effector offset with fixed orientation.
  /// \param[in] space The StateSpace for the metaskeleton.
  /// \param[in] metaSkeleton Metaskeleton to plan with.
  /// \param[in] body Bodynode for the end-effector.
  /// \param[in] collisionFree CollisionFree constraint to check.
  /// Self-collision is checked by default.
  /// \param[in] direction Direction unit vector in the world frame.
  /// \param[in] distance Distance distance to move, in meters.
  /// \param[in] timelimit Timelimit for planning.
  /// \param[in] positionTolerance Constraint tolerance in meters.
  /// \param[in] angularTolerance Constraint tolerance in radians.
  /// \return Output trajectory
  trajectory::TrajectoryPtr planToEndEffectorOffset(
      const statespace::dart::MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& body,
      const constraint::dart::CollisionFreePtr& collisionFree,
      const Eigen::Vector3d& direction,
      double distance,
      double timelimit,
      double positionTolerance,
      double angularTolerance);

  /// TODO: Replace this with Problem interface.
  /// Plans to a desired end-effector offset along the z axis of the
  /// end-effector.
  /// \param[in] space The StateSpace for the metaskeleton.
  /// \param[in] metaSkeleton Metaskeleton to plan with.
  /// \param[in] body Bodynode for the end-effector.
  /// \param[in] collisionFree CollisionFree constraint to check.
  /// Self-collision is checked by default.
  /// \param[in] distance Distance distance to move, in meters.
  /// \param[in] timelimit Timelimit for plannnig.
  /// \param[in] positionTolerance Constraint tolerance in meters.
  /// \param[in] angularTolerance Constraint tolerance in radians.
  /// \return Output trajectory
  trajectory::TrajectoryPtr planEndEffectorStraight(
      statespace::dart::MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& body,
      const constraint::dart::CollisionFreePtr& collisionFree,
      double distance,
      double timelimit,
      double positionTolerance,
      double angularTolerance);

  /// TODO: This should be revisited once we have Planner API.
  /// Sets VectorFieldPlanner parameters.
  /// \param[in] vfParameters VectorField Parameters
  void setVectorFieldPlannerParameters(
      const util::VectorFieldPlannerParameters& vfParameters);

private:
  /// The robot whose metaSkeleton corresponds to this manipulator
  RobotPtr mRobot;

  /// Hand of this manipulator
  HandPtr mHand;

  /// VectorFieldPlanenr-related parameters
  util::VectorFieldPlannerParameters mVectorFieldParameters;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_CONCRETEMANIPULATOR_HPP_
