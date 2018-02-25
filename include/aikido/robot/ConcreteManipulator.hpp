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
  /// Constructor.
  /// \param[in] robot Robot corresponding to this manipulator.
  /// \param[in] hand Hand of this manipulator.
  ConcreteManipulator(RobotPtr robot, HandPtr hand);

  virtual ~ConcreteManipulator() = default;

  // Documentation inherited.
  virtual HandPtr getHand() override;

  // Documentation inherited.
  virtual std::unique_ptr<aikido::trajectory::Spline> smoothPath(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const aikido::trajectory::Trajectory* path,
      const constraint::TestablePtr& constraint) override;

  // Documentation inherited.
  virtual std::unique_ptr<aikido::trajectory::Spline> retimePath(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const aikido::trajectory::Trajectory* path) override;

  // Documentation inherited.
  virtual std::future<void> executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) override;

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
  virtual dart::dynamics::MetaSkeletonPtr getMetaSkeleton() override;

  // Documentation inherited.
  virtual aikido::statespace::dart::MetaSkeletonStateSpacePtr
    getStateSpace() const override;

  // Documentation inherited.
  virtual void setRoot(Robot* robot) override;

  // Documentation inherited.
  virtual void step(
      const std::chrono::system_clock::time_point& timepoint) override;

  // Documentation inherited.
  virtual constraint::dart::CollisionFreePtr getSelfCollisionConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton) override;

  // Documentation inherited.
  virtual aikido::constraint::TestablePtr getFullCollisionConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const constraint::dart::CollisionFreePtr& collisionFree) override;

  /// TODO: Replace this with Problem interface.
  /// Plans to a desired end-effector offset with fixed orientation.
  /// \param[in] space The StateSpace for the metaskeleton.
  /// \param[in] metaSkeleton Metaskeleton to plan with.
  /// \param[in] body Bodynode for the end effector.
  /// \param[in] collisionFree CollisionFree constraint to check.
  /// Self-collision is checked by default.
  /// \param[in] direction Direction unit vector in the world frame.
  /// \param[in] distance Distance distance to move, in meters.
  /// \param[in] timelimit Timelimit for planning
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
  /// \param[in] body Bodynode for the end effector.
  /// \param[in] collisionFree CollisionFree constraint to check.
  /// Self-collision is checked by default.
  /// \param[in] distance Distance distance to move, in meters.
  /// \param[in] timelimit Timelimit for plannnig.
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

  /// TODO: This should be revisited once we have Planner API.
  /// Sets CRRTPlanner parameters.
  /// \param[in] crrtParameters CRRT planner parameters
  void setCRRTPlannerParameters(
      const util::CRRTPlannerParameters& crrtParameters);

private:
  RobotPtr mRobot;
  HandPtr mHand;
  util::VectorFieldPlannerParameters mVectorFieldParameters;
  util::CRRTPlannerParameters mCRRTParameters;

  /// Returns the direction of an end effector (along z axis) in the world frame
  /// \param body Bodynode for the end effector
  /// \return The direction of the end effector (z axis of the frame)
  Eigen::Vector3d getEndEffectorDirection(
      const dart::dynamics::BodyNodePtr& body) const;

};

} // namespace robot
} // namespace aikido

#endif
