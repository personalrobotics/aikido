#ifndef AIKIDO_CONSTRAINT_DART_HPP_
#define AIKIDO_CONSTRAINT_DART_HPP_
#include "Differentiable.hpp"
#include "Projectable.hpp"
#include "Sampleable.hpp"
#include "TestableConstraint.hpp"
#include "../statespace/dart/JointStateSpace.hpp"


namespace aikido {
namespace constraint {

struct GenericJointBoundConstraint
{
  using StateSpace = statespace::JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> createDifferentiable(
    StateSpacePtr _stateSpace);

  static std::unique_ptr<Projectable> createProjectable(
    StateSpacePtr _stateSpace);

  static std::unique_ptr<SampleableConstraint> createUniformSampleable(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng);

  static std::unique_ptr<TestableConstraint> createTestable(
    StateSpacePtr _stateSpace);
};

} // namespace constraint
} // namespace aikido

#include "detail/dart.hpp"

#endif // ifndef AIKIDO_CONSTRAINT_DART_HPP_
