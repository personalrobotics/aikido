namespace aikido {
namespace robot {

//==============================================================================
template <typename T>
std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
ConcreteRobot::getTrajectoryPostProcessor(
    const Eigen::VectorXd& velocityLimits,
    const Eigen::VectorXd& accelerationLimits,
    const typename T::Params& postProcessorParams) const
{
  static_assert(
      std::is_base_of<aikido::planner::TrajectoryPostProcessor, T>::value,
      "T must derive from aikido::planner::TrajectoryPostProcessor");

  return std::make_shared<T>(
      velocityLimits, accelerationLimits, postProcessorParams);
}

//==============================================================================
template <typename T>
aikido::trajectory::UniqueSplinePtr ConcreteRobot::postProcessPath(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path,
    const constraint::TestablePtr& constraint,
    const typename T::Params& postProcessorParams)
{
  return postProcessPath(
      getVelocityLimits(*metaSkeleton),
      getAccelerationLimits(*metaSkeleton),
      path,
      constraint,
      postProcessorParams);
}

//==============================================================================
template <typename T>
aikido::trajectory::UniqueSplinePtr ConcreteRobot::postProcessPath(
    const Eigen::VectorXd& velocityLimits,
    const Eigen::VectorXd& accelerationLimits,
    const aikido::trajectory::Trajectory* path,
    const constraint::TestablePtr& constraint,
    const typename T::Params& postProcessorParams)
{
  auto postProcessor = getTrajectoryPostProcessor(
      velocityLimits, accelerationLimits, postProcessorParams);

  auto interpolated
      = dynamic_cast<const aikido::trajectory::Interpolated*>(path);
  if (interpolated)
    return postProcessor->postprocess(
        *interpolated, *(cloneRNG().get()), constraint);

  auto spline = dynamic_cast<const aikido::trajectory::Spline*>(path);
  if (spline)
    return postProcessor->postprocess(*spline, *(cloneRNG().get()), constraint);

  std::cerr
      << "[postProcessPath]: Path should be either Spline or Interpolated."
      << std::endl;
  return nullptr;
}

} // namespace robot
} // namespace aikido
