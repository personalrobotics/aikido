#include <aikido/statespace/RealVectorJointStateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpaceSampleableConstraint.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
RealVectorJointStateSpace::RealVectorJointStateSpace(
      dart::dynamics::Joint* _joint)
  : RealVectorStateSpace(_joint->getNumDofs())
  , JointStateSpace(_joint)
{
}

//=============================================================================
void RealVectorJointStateSpace::getState(StateSpace::State* _state) const
{
  setValue(static_cast<State*>(_state), mJoint->getPositions());
}

//=============================================================================
void RealVectorJointStateSpace::setState(const StateSpace::State* _state) const
{
  mJoint->setPositions(getValue(static_cast<const State*>(_state)));
}

//=============================================================================
auto RealVectorJointStateSpace::createSampleableConstraint(
  std::unique_ptr<util::RNG> _rng) const -> SampleableConstraintPtr
{
  Eigen::VectorXd lowerLimits(getDimension());
  Eigen::VectorXd upperLimits(getDimension());

  for (size_t i = 0; i < getDimension(); ++i)
  {
    lowerLimits[i] = mJoint->getPositionLowerLimit(i);
    upperLimits[i] = mJoint->getPositionUpperLimit(i);
  }

  return std::make_shared<RealVectorStateSpaceSampleableConstraint>(
    // TODO: SampleableConstraint should operate on `const StateSpace`.
    std::const_pointer_cast<RealVectorJointStateSpace>(shared_from_this()),
    std::move(_rng), lowerLimits, upperLimits);
}

} // namespace statespace
} // namespace aikido
