#ifndef AIKIDO_CONSTRAINT_FINITESAMPLECONSTRAINT_H
#define AIKIDO_CONSTRAINT_FINITESAMPLECONSTRAINT_H

#include "Sampleable.hpp"

namespace aikido {
namespace constraint {

// Constraint that always returns a finite set of samples.
// Its SampleGenerator will generate sample in the order of _states
// until all samples are exhausted.
class FiniteSampleConstraint : public SampleableConstraint
{
public:
  FiniteSampleConstraint(
    statespace::StateSpacePtr _stateSpace,
    const statespace::StateSpace::State* _state);

  FiniteSampleConstraint(
    statespace::StateSpacePtr _stateSpace,
    const std::vector<const statespace::StateSpace::State*>& _states);

  FiniteSampleConstraint(const FiniteSampleConstraint& other) = delete;
  FiniteSampleConstraint(FiniteSampleConstraint&& other) = delete;

  FiniteSampleConstraint& operator=(
    const FiniteSampleConstraint& other) = delete;
  FiniteSampleConstraint& operator=(
    FiniteSampleConstraint&& other) = delete;

  virtual ~FiniteSampleConstraint();

  /// Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  statespace::StateSpacePtr mStateSpace;
  std::vector<statespace::StateSpace::State*> mStates;

};


}
}

#endif
