#include <aikido/constraint/dart.hpp>

namespace aikido {
namespace constraint {

std::unique_ptr<Differentiable> createDifferentiableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace)
{
  return detail::ForOneOf<
      detail::createDifferentiableFor_impl,
      statespace::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace));
}

std::unique_ptr<Projectable> createProjectableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace)
{
  return detail::ForOneOf<
      detail::createProjectableFor_impl,
      statespace::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace));
}

std::unique_ptr<TestableConstraint> createTestableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace)
{
  return detail::ForOneOf<
      detail::createTestableFor_impl,
      statespace::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace));
}

std::unique_ptr<SampleableConstraint> createSampleableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace,
  std::unique_ptr<util::RNG> _rng)
{
  return detail::ForOneOf<
      detail::createSampleableFor_impl,
      statespace::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace), std::move(_rng));
}

} // namespace constraint
} // namespace aikido
